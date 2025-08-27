classdef Drone < handle
    % Drone - Class representing the VTOL UAV
    properties
        % Physical properties
        position = [0, 0, 20]      % Initial position [x, y, z] (TRUE state)
        velocity = [0, 0, 0]       % Velocity vector [vx, vy, vz] (TRUE state)
        orientation = [0, 0, 0]    % [roll, pitch, yaw] in radians (TRUE state)
        % Control properties
        targetPosition = [0, 0, 20]
        targetVelocity = [0, 0, 0]
        
        %%% ADDED FOR SEARCH ALGORITHM %%%
        targetYaw = NaN % Target yaw angle (NaN means not actively controlled)
        
        descentRate = 0
        % Drone parameters
        maxSpeed = 10              % Maximum speed in m/s
        maxAcceleration = 5        % Maximum acceleration in m/s^2
        maxRotationRate = pi/4     % Maximum rotation rate in rad/s
        % Sensors
        camera
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
                          'distanceToTarget', Inf, 'arUcoDetections', [], ...
                          'ekf_altitude', 0, 'ekf_yaw', 0)
        % Simulated sensor measurements
        gpsPosition
        imuAcceleration
        imuAngularVelocity
        barometerAltitude
        % Sensor noise parameters
        gpsNoiseStd = 1.5
        imuAccelNoiseStd = 0.2
        imuGyroNoiseStd = 0.01
        baroNoiseStd = 0.5
        % Sensor bias
        imuAccelBias = [0, 0, 0]
        imuGyroBias = [0, 0, 0]
        baroBias = 0
        
        %% LQR: Controller Property
        lqr_gain_K              % The optimal gain matrix calculated by LQR
        % EKF: EKF Properties
        ekf_estimated_state
        ekf_covariance_matrix
        process_noise
        measurement_noise
    end
    methods
        function obj = Drone(environment)
            % Constructor
            obj.environment = environment;
            obj.camera = Camera(obj);
            % EKF: Initialize the Extended Kalman Filter
            obj.ekf_estimated_state = [obj.position, obj.velocity, obj.orientation]';
            obj.ekf_covariance_matrix = eye(9) * 0.1;
            obj.process_noise = diag([0.05, 0.05, 0.05, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01]);
            obj.measurement_noise = diag([obj.gpsNoiseStd^2, obj.gpsNoiseStd^2, obj.baroNoiseStd^2, obj.imuGyroNoiseStd^2, obj.imuGyroNoiseStd^2, obj.imuGyroNoiseStd^2]);
            
            %% LQR: Define the State-Space Model and Cost Matrices
            A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
            B = [0 0 0; 0 0 0; 0 0 0; 1 0 0; 0 1 0; 0 0 1];
            Q = diag([10, 10, 10, 1, 1, 1]);
            R = eye(3) * 0.1;
            obj.lqr_gain_K = lqr(A, B, Q, R);
        end
        
        function update(obj, dt, wind_vector)
            % 1. Run sensor simulation first to get measurements
            obj.updateSensors(dt);
            % 2. Run the EKF to get the best state estimate
            obj.runEKF(dt);
            % 3. Use the EKF's ESTIMATE for control decisions
            estimated_pos = obj.ekf_estimated_state(1:3)';
            estimated_vel = obj.ekf_estimated_state(4:6)';
            estimated_ori = obj.ekf_estimated_state(7:9)';
            [acceleration, rotationRates] = obj.calculateControlInputs(estimated_pos, estimated_vel, estimated_ori);
            
            % 4. Apply dynamics to the TRUE state for simulation
            obj.applyDynamics(acceleration, rotationRates, dt, wind_vector);
            
            obj.updateRotorSpeeds(acceleration, rotationRates);
            obj.applyEnvironmentalConstraints();
            % 5. Update telemetry for dashboards
            obj.updateTelemetry();
            obj.telemetry.batteryLevel = max(0, obj.telemetry.batteryLevel - 0.01 * dt);
        end
        function updateTelemetry(obj)
            obj.telemetry.altitude = obj.position(3);
            obj.telemetry.speed = norm(obj.velocity);
            obj.telemetry.roll = obj.orientation(1) * 180/pi;
            obj.telemetry.pitch = obj.orientation(2) * 180/pi;
            obj.telemetry.yaw = obj.orientation(3) * 180/pi;
            obj.telemetry.ekf_altitude = obj.ekf_estimated_state(3);
            obj.telemetry.ekf_yaw = obj.ekf_estimated_state(9) * 180/pi;
        end
        
        function setTargetPosition(obj, position)
            % Sets the drone's target position for the controller
            obj.targetPosition = position;
        end
        function setTargetVelocity(obj, velocity)
            % Sets the drone's target velocity for the controller
            obj.targetVelocity = velocity;
        end
        
        %%% ADDED TO FIX ERROR %%%
        function setTargetYaw(obj, yaw)
            % Sets a specific target heading for the drone
            obj.targetYaw = yaw;
        end
        
        function setDescentRate(obj, rate)
            % Sets the drone's descent rate
            obj.descentRate = rate;
        end
        
        %%% MODIFIED TO FIX ERROR %%%
        function [acceleration, rotationRates] = calculateControlInputs(obj, currentPosition, currentVelocity, currentOrientation)
            % --- LQR CONTROLLER ---
            posError = currentPosition - obj.targetPosition;
            velError = currentVelocity - obj.targetVelocity;
            state_error = [posError'; velError']; 
            desiredAcceleration = -obj.lqr_gain_K * state_error;
            desiredAcceleration = desiredAcceleration';
            
            if obj.descentRate > 0
                vertical_vel_error = -obj.descentRate - currentVelocity(3);
                desiredAcceleration(3) = vertical_vel_error * 5.0; 
            end
            
            accelerationMagnitude = norm(desiredAcceleration);
            if accelerationMagnitude > obj.maxAcceleration
                desiredAcceleration = desiredAcceleration * (obj.maxAcceleration / accelerationMagnitude);
            end
            
            desiredRoll = atan2(desiredAcceleration(2), 9.81);
            desiredPitch = -atan2(desiredAcceleration(1), 9.81);
            
            % Use the specific target yaw if it's set, otherwise point towards the target position.
            if ~isnan(obj.targetYaw)
                desiredYaw = obj.targetYaw;
            else
                target_vector = obj.targetPosition - currentPosition;
                desiredYaw = atan2(target_vector(2), target_vector(1));
            end
            
            orientationError = [
                desiredRoll - currentOrientation(1), ...
                desiredPitch - currentOrientation(2), ...
                desiredYaw - currentOrientation(3)
            ];
            orientationError(3) = mod(orientationError(3) + pi, 2*pi) - pi;
            kpOrientation = 5.0;
            rotationRates = kpOrientation * orientationError;
            
            rotationMagnitude = norm(rotationRates);
            if rotationMagnitude > obj.maxRotationRate
                rotationRates = rotationRates * (obj.maxRotationRate / rotationMagnitude);
            end
            
            acceleration = desiredAcceleration;
        end
        
        function applyDynamics(obj, acceleration, rotationRates, dt, wind_vector)
            obj.velocity = obj.velocity + acceleration * dt;
            drag_coefficient = 0.1;
            wind_force = drag_coefficient * (wind_vector - obj.velocity);
            obj.velocity = obj.velocity + wind_force * dt;
            velocityMagnitude = norm(obj.velocity);
            if velocityMagnitude > obj.maxSpeed
                obj.velocity = obj.velocity * (obj.maxSpeed / velocityMagnitude);
            end
            obj.position = obj.position + obj.velocity * dt;
            obj.orientation = obj.orientation + rotationRates * dt;
            obj.orientation(1) = max(min(obj.orientation(1), pi/4), -pi/4);
            obj.orientation(2) = max(min(obj.orientation(2), pi/4), -pi/4);
            obj.orientation(3) = mod(obj.orientation(3), 2*pi);
        end
        
        function updateRotorSpeeds(obj, acceleration, rotationRates)
            baseThrust = 9.81 / 4;
            rollThrust = rotationRates(1) * 0.5;
            pitchThrust = rotationRates(2) * 0.5;
            yawThrust = rotationRates(3) * 0.2;
            verticalThrust = acceleration(3) / 4;
            
            obj.rotorSpeeds(1) = baseThrust + verticalThrust - pitchThrust - rollThrust - yawThrust;
            obj.rotorSpeeds(2) = baseThrust + verticalThrust - pitchThrust + rollThrust + yawThrust;
            obj.rotorSpeeds(3) = baseThrust + verticalThrust + pitchThrust - rollThrust + yawThrust;
            obj.rotorSpeeds(4) = baseThrust + verticalThrust + pitchThrust + rollThrust - yawThrust;
            
            obj.rotorSpeeds = max(obj.rotorSpeeds, 0);
        end
        function applyEnvironmentalConstraints(obj)
            if obj.position(3) < 0.5
                obj.position(3) = 0.5;
                obj.velocity(3) = max(0, obj.velocity(3));
            end
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
            obstacles = obj.environment.getObstacles();
            for i = 1:size(obstacles, 1)
                obstaclePos = obstacles(i, 1:3);
                obstacleRadius = obstacles(i, 4);
                vec = obj.position - obstaclePos;
                dist = norm(vec);
                safetyMargin = 1.0;
                if dist < (obstacleRadius + obj.bodyRadius + safetyMargin)
                    repulsionStrength = 5.0 * (1.0 - dist / (obstacleRadius + obj.bodyRadius + safetyMargin));
                    repulsionForce = vec / dist * repulsionStrength;
                    obj.velocity = obj.velocity + repulsionForce;
                end
            end
        end
        
        function drawDrone(obj, ax)
            R = obj.getRotationMatrix();
            [X, Y, Z] = sphere(20);
            X = X * obj.bodyRadius + obj.position(1);
            Y = Y * obj.bodyRadius + obj.position(2);
            Z = Z * obj.bodyRadius + obj.position(3);
            surf(ax, X, Y, Z, 'FaceColor', [0.2, 0.2, 0.8], ...
                 'EdgeColor', 'none', 'FaceAlpha', 0.8);
            
            for i = 1:size(obj.rotorPositions, 1)
                rotorPos = obj.rotorPositions(i, :);
                worldRotorPos = obj.position + (R * rotorPos')';
                
                [X_cyl, Y_cyl, Z_cyl] = cylinder(obj.rotorRadius, 20);
                Z_cyl = Z_cyl * obj.rotorHeight;
                
                cyl_verts = [X_cyl(:)'; Y_cyl(:)'; Z_cyl(:)'];
                rotated_verts = R * cyl_verts;
                
                X_rot = reshape(rotated_verts(1,:), size(X_cyl)) + worldRotorPos(1);
                Y_rot = reshape(rotated_verts(2,:), size(Y_cyl)) + worldRotorPos(2);
                Z_rot = reshape(rotated_verts(3,:), size(Z_cyl)) + worldRotorPos(3);
                surf(ax, X_rot, Y_rot, Z_rot, 'FaceColor', [0.8, 0.2, 0.2], ...
                     'EdgeColor', 'none');
            end
            
            directionVector = (R * [1; 0; 0])' * obj.bodyRadius * 1.5;
            arrowEnd = obj.position + directionVector;
            
            line(ax, [obj.position(1), arrowEnd(1)], ...
                 [obj.position(2), arrowEnd(2)], ...
                 [obj.position(3), arrowEnd(3)], ...
                 'Color', 'k', 'LineWidth', 2);
        end
        function R = getRotationMatrix(obj)
            roll = obj.orientation(1);
            pitch = obj.orientation(2);
            yaw = obj.orientation(3);
            
            Rx = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)];
            Ry = [cos(pitch), 0, sin(pitch); 0, 1, 0; -sin(pitch), 0, cos(pitch)];
            Rz = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
            R = Rz * Ry * Rx;
        end
        
        function updateSensors(obj, dt)
            obj.gpsPosition = obj.position + obj.gpsNoiseStd * randn(1, 3);
            persistent previousVelocity
            if isempty(previousVelocity), previousVelocity = obj.velocity; end
            trueAccel = (obj.velocity - previousVelocity) / dt;
            previousVelocity = obj.velocity;
            obj.imuAcceleration = trueAccel + obj.imuAccelBias + obj.imuAccelNoiseStd * randn(1, 3);
            persistent previousOrientation
            if isempty(previousOrientation), previousOrientation = obj.orientation; end
            trueGyro = (obj.orientation - previousOrientation) / dt;
            previousOrientation = obj.orientation;
            obj.imuAngularVelocity = trueGyro + obj.imuGyroBias + obj.imuGyroNoiseStd * randn(1, 3);
            obj.barometerAltitude = obj.position(3) + obj.baroBias + obj.baroNoiseStd * randn();
        end
    end
    
    methods (Access = private)
        function obj = runEKF(obj, dt)
            F = eye(9);
            F(1:3, 4:6) = eye(3) * dt;
            obj.ekf_estimated_state = F * obj.ekf_estimated_state;
            obj.ekf_covariance_matrix = F * obj.ekf_covariance_matrix * F' + obj.process_noise;
            z = [obj.gpsPosition(1); obj.gpsPosition(2); obj.barometerAltitude; obj.imuAngularVelocity(1); obj.imuAngularVelocity(2); obj.imuAngularVelocity(3)];
            H = [1 0 0 0 0 0 0 0 0; 0 1 0 0 0 0 0 0 0; 0 0 1 0 0 0 0 0 0; 0 0 0 0 0 0 1 0 0; 0 0 0 0 0 0 0 1 0; 0 0 0 0 0 0 0 0 1];
            y = z - H * obj.ekf_estimated_state;
            S = H * obj.ekf_covariance_matrix * H' + obj.measurement_noise;
            K = obj.ekf_covariance_matrix * H' / S;
            obj.ekf_estimated_state = obj.ekf_estimated_state + K * y;
            obj.ekf_covariance_matrix = (eye(9) - K * H) * obj.ekf_covariance_matrix;
        end
    end
end
