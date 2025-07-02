classdef Drone < handle
    % Drone - Class representing the VTOL UAV
    
    properties
        % Physical properties
        position = [0, 0, 20]  % Initial position [x, y, z]
        velocity = [0, 0, 0]   % Velocity vector [vx, vy, vz]
        orientation = [0, 0, 0] % Orientation [roll, pitch, yaw] in radians
        
        % Control properties
        targetPosition = [0, 0, 20]
        targetVelocity = [0, 0, 0]
        descentRate = 0
        
        % Drone parameters
        maxSpeed = 10          % Maximum speed in m/s
        maxAcceleration = 5    % Maximum acceleration in m/s²
        maxRotationRate = pi/4 % Maximum rotation rate in rad/s
        
        % Sensors
        camera
        
        % Environment reference
        environment
        
        % Landing status
        arUcoAuthenticated = false
        authenticatedArUcoID = 0
        
        % Drone model properties
        bodyRadius = 0.5
        rotorRadius = 0.2
        rotorHeight = 0.05
        rotorPositions = [
            0.9, 0.9, 0;
            0.9, -0.9, 0;
            -0.9, 0.9, 0;
            -0.9, -0.9, 0
        ]
        rotorSpeeds = zeros(4, 1)
        
        % Telemetry data for dashboard
        telemetry = struct('altitude', 0, 'speed', 0, 'batteryLevel', 100, ...
                          'roll', 0, 'pitch', 0, 'yaw', 0, ...
                          'distanceToTarget', Inf, 'arUcoDetections', [])
    end
    
    methods
        function obj = Drone(environment)
            % Constructor
            obj.environment = environment;
            
            % Initialize camera
            obj.camera = Camera(obj);
        end
        
        function update(obj, dt)
            % Update drone state
            
            % Calculate control inputs
            [acceleration, rotationRates] = obj.calculateControlInputs();
            
            % Apply dynamics
            obj.applyDynamics(acceleration, rotationRates, dt);
            
            % Update rotor speeds based on control inputs
            obj.updateRotorSpeeds(acceleration, rotationRates);
            
            % Apply environmental constraints
            obj.applyEnvironmentalConstraints();
            
            % Update telemetry data
            obj.updateTelemetry();
            
            % Simulate battery drain
            obj.telemetry.batteryLevel = max(0, obj.telemetry.batteryLevel - 0.01 * dt);
        end
        
        function updateTelemetry(obj)
            % Update telemetry data for dashboard
            obj.telemetry.altitude = obj.position(3);
            obj.telemetry.speed = norm(obj.velocity);
            obj.telemetry.roll = obj.orientation(1) * 180/pi;
            obj.telemetry.pitch = obj.orientation(2) * 180/pi;
            obj.telemetry.yaw = obj.orientation(3) * 180/pi;
            
            % Distance to target is updated externally
        end
        
        function [acceleration, rotationRates] = calculateControlInputs(obj)
            % Calculate control inputs based on target position and velocity
            
            % Position error
            posError = obj.targetPosition - obj.position;
            
            % Velocity error
            velError = obj.targetVelocity - obj.velocity;
            
            % PD controller for position
            kp = 1.0;  % Proportional gain
            kd = 2.0;  % Derivative gain
            
            % Calculate desired acceleration
            desiredAcceleration = kp * posError + kd * velError;
            
            % Apply descent rate for landing
            if obj.descentRate > 0
                desiredAcceleration(3) = desiredAcceleration(3) - obj.descentRate;
            end
            
            % Limit acceleration magnitude
            accelerationMagnitude = norm(desiredAcceleration);
            if accelerationMagnitude > obj.maxAcceleration
                desiredAcceleration = desiredAcceleration * (obj.maxAcceleration / accelerationMagnitude);
            end
            
            % Calculate desired orientation
            desiredRoll = atan2(desiredAcceleration(2), 9.81);
            desiredPitch = -atan2(desiredAcceleration(1), 9.81);
            
            % Calculate heading to target
            dx = posError(1);
            dy = posError(2);
            desiredYaw = atan2(dy, dx);
            
            % Calculate orientation error
            orientationError = [
                desiredRoll - obj.orientation(1),
                desiredPitch - obj.orientation(2),
                desiredYaw - obj.orientation(3)
            ];
            
            % Normalize yaw error to [-pi, pi]
            orientationError(3) = mod(orientationError(3) + pi, 2*pi) - pi;
            
            % PD controller for orientation
            kpOrientation = 5.0;
            rotationRates = kpOrientation * orientationError;
            
            % Limit rotation rates
            rotationMagnitude = norm(rotationRates);
            if rotationMagnitude > obj.maxRotationRate
                rotationRates = rotationRates * (obj.maxRotationRate / rotationMagnitude);
            end
            
            % Return control inputs
            acceleration = desiredAcceleration;
        end
        
        function applyDynamics(obj, acceleration, rotationRates, dt)
            % Apply dynamics to update position, velocity, and orientation
            
            % Update velocity
            obj.velocity = obj.velocity + acceleration * dt;
            
            % Limit velocity magnitude
            velocityMagnitude = norm(obj.velocity);
            if velocityMagnitude > obj.maxSpeed
                obj.velocity = obj.velocity * (obj.maxSpeed / velocityMagnitude);
            end
            
            % Update position
            obj.position = obj.position + obj.velocity * dt;
            
            % Update orientation
            obj.orientation = obj.orientation + rotationRates * dt;
            
            % Normalize orientation
            obj.orientation(1) = max(min(obj.orientation(1), pi/4), -pi/4);  % Limit roll
            obj.orientation(2) = max(min(obj.orientation(2), pi/4), -pi/4);  % Limit pitch
            obj.orientation(3) = mod(obj.orientation(3), 2*pi);              % Wrap yaw
        end
        
        function updateRotorSpeeds(obj, acceleration, rotationRates)
            % Update rotor speeds based on control inputs
            
            % Base thrust (hover)
            baseThrust = 9.81 / 4;
            
            % Calculate differential thrusts
            rollThrust = rotationRates(1) * 0.5;
            pitchThrust = rotationRates(2) * 0.5;
            yawThrust = rotationRates(3) * 0.2;
            
            % Vertical acceleration contribution
            verticalThrust = acceleration(3) / 4;
            
            % Calculate individual rotor thrusts
            obj.rotorSpeeds(1) = baseThrust + verticalThrust - pitchThrust - rollThrust - yawThrust;
            obj.rotorSpeeds(2) = baseThrust + verticalThrust - pitchThrust + rollThrust + yawThrust;
            obj.rotorSpeeds(3) = baseThrust + verticalThrust + pitchThrust - rollThrust + yawThrust;
            obj.rotorSpeeds(4) = baseThrust + verticalThrust + pitchThrust + rollThrust - yawThrust;
            
            % Ensure non-negative rotor speeds
            obj.rotorSpeeds = max(obj.rotorSpeeds, 0);
        end
        
        function applyEnvironmentalConstraints(obj)
            % Apply environmental constraints
            
            % Ground collision
            if obj.position(3) < 0.5
                obj.position(3) = 0.5;
                obj.velocity(3) = max(0, obj.velocity(3));
            end
            
            % Boundary constraints
            bounds = obj.environment.getBounds();
            for i = 1:3
                if obj.position(i) < bounds(i, 1)
                    obj.position(i) = bounds(i, 1);
                    obj.velocity(i) = max(0, obj.velocity(i));
                elseif obj.position(i) > bounds(i, 2)
                    obj.position(i) = bounds(i, 2);
                    obj.velocity(i) = min(0, obj.velocity(i));
                end
            end
            
            % Obstacle avoidance (simplified)
            obstacles = obj.environment.getObstacles();
            for i = 1:size(obstacles, 1)
                obstaclePos = obstacles(i, 1:3);
                obstacleRadius = obstacles(i, 4);
                
                % Vector from obstacle to drone
                vec = obj.position - obstaclePos;
                dist = norm(vec);
                
                % Check for collision
                safetyMargin = 1.0;
                if dist < (obstacleRadius + obj.bodyRadius + safetyMargin)
                    % Calculate repulsion force
                    repulsionStrength = 5.0 * (1.0 - dist / (obstacleRadius + obj.bodyRadius + safetyMargin));
                    repulsionForce = vec / dist * repulsionStrength;
                    
                    % Apply repulsion
                    obj.velocity = obj.velocity + repulsionForce;
                end
            end
        end
        
        function setTargetPosition(obj, position)
            % Set target position
            obj.targetPosition = position;
        end
        
        function setTargetVelocity(obj, velocity)
            % Set target velocity
            obj.targetVelocity = velocity;
        end
        
        function setDescentRate(obj, rate)
            % Set descent rate for landing
            obj.descentRate = rate;
        end
        
        function manualControl(obj, direction)
            % Apply manual control input
            
            % Scale direction
            direction = direction * 5;
            
            % Transform direction to world coordinates based on drone orientation
            R = obj.getRotationMatrix();
            worldDirection = (R * direction')';
            
            % Set target position
            obj.targetPosition = obj.position + worldDirection;
        end
        
        function R = getRotationMatrix(obj)
            % Get rotation matrix from drone orientation
            
            % Extract Euler angles
            roll = obj.orientation(1);
            pitch = obj.orientation(2);
            yaw = obj.orientation(3);
            
            % Calculate rotation matrix
            Rx = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)];
            Ry = [cos(pitch), 0, sin(pitch); 0, 1, 0; -sin(pitch), 0, cos(pitch)];
            Rz = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
            
            % Combine rotations
            R = Rz * Ry * Rx;
        end
        
        function drawDrone(obj, ax)
            % Draw drone in the given axes
            
            % Get rotation matrix
            R = obj.getRotationMatrix();
            
            % Draw drone body
            [X, Y, Z] = sphere(20);
            X = X * obj.bodyRadius;
            Y = Y * obj.bodyRadius;
            Z = Z * obj.bodyRadius;
            
            % Transform to drone position
            X = X + obj.position(1);
            Y = Y + obj.position(2);
            Z = Z + obj.position(3);
            
            % Plot body
            surf(ax, X, Y, Z, 'FaceColor', [0.2, 0.2, 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.8);
            
            % Draw rotors
            for i = 1:size(obj.rotorPositions, 1)
                % Get rotor position in drone frame
                rotorPos = obj.rotorPositions(i, :);
                
                % Transform to world frame
                worldRotorPos = obj.position + (R * rotorPos')';
                
                % Create rotor cylinder
                [X, Y, Z] = cylinder(obj.rotorRadius, 20);
                Z = Z * obj.rotorHeight;
                
                % Transform to rotor position
                X = X + worldRotorPos(1);
                Y = Y + worldRotorPos(2);
                Z = Z + worldRotorPos(3);
                
                % Plot rotor
                surf(ax, X, Y, Z, 'FaceColor', [0.8, 0.2, 0.2], 'EdgeColor', 'none');
                
                % Draw rotor blades (simplified as lines)
                bladeLength = obj.rotorRadius * 2;
                
                % Calculate blade rotation based on rotor speed
                bladeAngle = mod(obj.rotorSpeeds(i) * 10, 2*pi);
                
                % Draw blades
                for angle = [0, pi] + bladeAngle
                    bladeEnd = worldRotorPos + [bladeLength * cos(angle), bladeLength * sin(angle), 0];
                    line(ax, [worldRotorPos(1), bladeEnd(1)], ...
                             [worldRotorPos(2), bladeEnd(2)], ...
                             [worldRotorPos(3), bladeEnd(3)], ...
                        'Color', [0.5, 0.5, 0.5], 'LineWidth', 2);
                end
            end
            
            % Draw direction indicator
            directionVector = (R * [1, 0, 0]')';
            arrowEnd = obj.position + directionVector * obj.bodyRadius * 1.5;
            line(ax, [obj.position(1), arrowEnd(1)], ...
                     [obj.position(2), arrowEnd(2)], ...
                     [obj.position(3), arrowEnd(3)], ...
                'Color', [0, 0, 0], 'LineWidth', 2);
        end
    end
end
