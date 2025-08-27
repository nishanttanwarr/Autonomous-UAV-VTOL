classdef Vehicle < handle
    % Vehicle - Class representing the moving vehicle with landing pad
    
    properties
        % --- Speed / longitudinal dynamics ---
        targetSpeed   double = 8.0;   % m/s (editable from UI)
        curSpeed      double = 0.0;   % m/s (derived)
        maxAccel      double = 2.0;   % m/s^2  (throttle accel limit)
        maxDecel      double = 3.0;   % m/s^2  (braking limit, positive number)
        maxJerk       double = 5.0;   % m/s^3  (rate limit on acceleration)
        tauSpeedCtl   double = 1.0;   % s      (P-like time const for speed ctrl)
        accPrev       double = 0.0;   % last applied longitudinal acceleration
        % --- Auto speed profile scheduling ---
        autoProfile   logical = true;
        segHoldT      double = 8.0;   % avg seconds per segment
        vMin          double = 0.0;   % min target speed
        vMax          double = 14.0;  % max target speed
        tSegEnd       double = 0.0;   % time when to pick next target
        % --- Existing fields ---
        environment
        position      double = [0 0 0];
        velocity      double = [8 0 0];
        orientation   double = 0;
        time          double = 0;
        leadVehicle
        followDistance double = 20;
        pathType      char = 'linear';
        pathParams
        length        double = 5;
        width         double = 2;
        height        double = 1.5;
        landingPadOffset double = [0 0 0];
        landingPadSize double = 2.0;
        arUcoSize     double = 1.0;
        arUcoID       double = 1;
    end
    
    methods
        function obj = Vehicle(environment)
            % Constructor
            obj.environment = environment;
            bounds = environment.getBounds();
            obj.position = [bounds(1,1) + 10, 0, 0];
            obj.orientation = atan2(obj.velocity(2), obj.velocity(1));
        end
        
        function update(obj, dt)
            % Auto profile scheduling for speed and direction
            if obj.autoProfile && obj.time >= obj.tSegEnd
                % Set a new random target speed
                obj.targetSpeed = obj.vMin + (obj.vMax - obj.vMin)*rand;
                holdT = 0.5*obj.segHoldT + obj.segHoldT*rand;
                obj.tSegEnd = obj.time + holdT;
                % 20% chance to simulate a near-stop
                if rand < 0.2
                    obj.targetSpeed = 0 + 2*rand;
                end
                
                if strcmp(obj.pathType, 'linear') && isempty(obj.leadVehicle)
                     turn_angle = (rand() - 0.5) * (pi/3);
                     rotation_matrix = [cos(turn_angle), -sin(turn_angle); 
                                        sin(turn_angle),  cos(turn_angle)];
                     current_direction = obj.velocity / (norm(obj.velocity) + 1e-10);
                     new_direction_2d = rotation_matrix * current_direction(1:2)';
                     obj.velocity(1:2) = new_direction_2d' * norm(obj.velocity);
                end
            end
            
            % Speed controller with jerk limit
            spdErr = obj.targetSpeed - obj.curSpeed;
            accCmd = spdErr / max(obj.tauSpeedCtl, 1e-3);
            accCmd = min(max(accCmd, -obj.maxDecel), obj.maxAccel);
            jerk = (accCmd - obj.accPrev) / max(dt, 1e-3);
            jerk = min(max(jerk, -obj.maxJerk), obj.maxJerk);
            a = obj.accPrev + jerk*dt;
            obj.accPrev = a;
            obj.curSpeed = max(0, obj.curSpeed + a*dt);
            
            % Update time
            obj.time = obj.time + dt;
            
            % If following another vehicle
            if ~isempty(obj.leadVehicle)
                obj.updateFollowing(dt);
            else
                % Update based on path type with current speed
                dir = obj.velocity / (norm(obj.velocity) + 1e-10);
                obj.velocity = dir * obj.curSpeed;
                switch obj.pathType
                    case 'linear'
                        obj.updateLinearPath(dt);
                    case 'circular'
                        obj.updateCircularPath(dt);
                    case 'sinusoidal'
                        obj.updateSinusoidalPath(dt);
                end
            end
            
            % Update orientation based on velocity
            if norm(obj.velocity(1:2)) > 1e-6
                obj.orientation = atan2(obj.velocity(2), obj.velocity(1));
            end
            
            % Apply environmental constraints
            obj.applyEnvironmentalConstraints();
        end
        
        function updateFollowing(obj, dt)
            leadPos = obj.leadVehicle.position;
            leadVel = obj.leadVehicle.velocity;
            leadDir = leadVel / (norm(leadVel) + 1e-10);
            desiredPos = leadPos - leadDir * obj.followDistance;
            posError = desiredPos - obj.position;
            kp = 1.0; kd = 2.0;
            desiredVel = kp * posError + leadVel;
            maxSpeed = 10;
            velMagnitude = norm(desiredVel);
            if velMagnitude > maxSpeed
                desiredVel = desiredVel * (maxSpeed / velMagnitude);
            end
            obj.velocity = desiredVel;
            obj.position = obj.position + obj.velocity * dt;
        end
        
        function updateLinearPath(obj, dt)
            obj.position = obj.position + obj.velocity * dt;
            bounds = obj.environment.getBounds();
            if obj.position(1) <= bounds(1,1) || obj.position(1) >= bounds(1,2)
                obj.velocity(1) = -obj.velocity(1);
            end
            if obj.position(2) <= bounds(2,1) || obj.position(2) >= bounds(2,2)
                obj.velocity(2) = -obj.velocity(2);
            end
        end
        
        function updateCircularPath(obj, dt)
            if ~isfield(obj.pathParams, 'radius'), obj.pathParams.radius = 50; end
            if ~isfield(obj.pathParams, 'center'), obj.pathParams.center = [0,0]; end
            angularVelocity = obj.curSpeed / obj.pathParams.radius;
            angle = obj.time * angularVelocity;
            center = obj.pathParams.center;
            obj.position(1) = center(1) + obj.pathParams.radius * cos(angle);
            obj.position(2) = center(2) + obj.pathParams.radius * sin(angle);
            obj.velocity(1) = -obj.pathParams.radius * angularVelocity * sin(angle);
            obj.velocity(2) =  obj.pathParams.radius * angularVelocity * cos(angle);
        end
        
        function updateSinusoidalPath(obj, dt)
            if isnan(obj.orientation), obj.orientation = 0; end
            forwardDir = [cos(obj.orientation), sin(obj.orientation), 0];
            lateralDir = [-sin(obj.orientation), cos(obj.orientation), 0];
            lateralAmplitude = 20;
            if ~isfield(obj.pathParams,'frequency'), obj.pathParams.frequency = 0.05; end
            lateralOffset = lateralAmplitude * sin(obj.time * obj.pathParams.frequency * 2 * pi);
            basePos = obj.position + forwardDir * obj.curSpeed * dt;
            center_2d = reshape(obj.pathParams.center, 1, []);
            center_3d = [center_2d, 0];
            targetLateralPos = center_3d + lateralDir * lateralOffset;
            lateralError = targetLateralPos - obj.position;
            lateralCorrection = lateralError * 0.1;
            obj.position = basePos + lateralCorrection;
            obj.velocity = forwardDir * obj.curSpeed + lateralCorrection / max(dt, 1e-6);
        end
        
        function applyEnvironmentalConstraints(obj)
            bounds = obj.environment.getBounds();
            for i = 1:2
                if obj.position(i) < bounds(i, 1) + obj.length/2
                    obj.position(i) = bounds(i, 1) + obj.length/2;
                    obj.velocity(i) = abs(obj.velocity(i));
                elseif obj.position(i) > bounds(i, 2) - obj.length/2
                    obj.position(i) = bounds(i, 2) - obj.length/2;
                    obj.velocity(i) = -abs(obj.velocity(i));
                end
            end
        end
        
        function padPos = getLandingPadPosition(obj)
            R = [cos(obj.orientation), -sin(obj.orientation), 0;
                 sin(obj.orientation), cos(obj.orientation), 0;
                 0, 0, 1];
            padPos = obj.position + (R * obj.landingPadOffset')';
        end
        
        %%% ADDED BACK TO FIX ERROR %%%
        function drawVehicle(obj, ax)
            % Draws the vehicle body and landing pad in the simulation
            
            % Define vehicle body vertices using a helper function
            [X, Y, Z] = cuboid(obj.position - [obj.length/2, obj.width/2, 0], ...
                               [obj.length, obj.width, obj.height]);
            
            % Create a patch object for the vehicle body
            patch(ax, X, Y, Z, [0.7 0.7 0.8], 'FaceAlpha', 0.9, 'EdgeColor', 'k');
            
            % Draw the landing pad on top of the vehicle
            obj.drawArUcoPattern(ax);
        end

        function drawArUcoPattern(obj, ax)
            % Draws the ArUco marker on the landing pad
            padPos = obj.getLandingPadPosition();
            s = obj.arUcoSize / 2;
            
            % Define vertices for the ArUco marker (a simple black square)
            verts = [padPos(1)-s, padPos(2)-s, padPos(3)+0.01;
                     padPos(1)+s, padPos(2)-s, padPos(3)+0.01;
                     padPos(1)+s, padPos(2)+s, padPos(3)+0.01;
                     padPos(1)-s, padPos(2)+s, padPos(3)+0.01];
            
            % Rotate vertices with the vehicle's orientation
            R = [cos(obj.orientation) -sin(obj.orientation); 
                 sin(obj.orientation) cos(obj.orientation)];
            rotated_verts = (R * (verts(:,1:2) - padPos(1:2))')' + padPos(1:2);
            
            % Draw the black square
            patch(ax, rotated_verts(:,1), rotated_verts(:,2), verts(:,3), 'k');
            
            % Draw the ID number in the center
            text(ax, padPos(1), padPos(2), padPos(3) + 0.1, num2str(obj.arUcoID), ...
                'HorizontalAlignment', 'center', 'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold');
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    end
end
