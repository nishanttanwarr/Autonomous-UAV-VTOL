classdef Vehicle < handle
    % Vehicle - Class representing the moving vehicle with landing pad
    
    properties
        % Physical properties
        position = [0, 0, 0]   % Position [x, y, z]
        velocity = [5, 0, 0]   % Velocity [vx, vy, vz]
        orientation = 0        % Orientation (yaw) in radians
        
        % Vehicle parameters
        length = 5
        width = 2
        height = 1.5
        
        % Landing pad properties
        landingPadSize = 1.5
        landingPadOffset = [0, 0, 1.5]  % Offset from vehicle center
        
        % ArUco marker properties
        arUcoID = 1           % ArUco marker ID (1, 2, or 3)
        arUcoSize = 0.5       % Size of ArUco marker
        
        % Path properties
        pathType = 'linear'    % 'linear', 'circular', 'sinusoidal'
        pathParams = struct('center', [0, 0, 0], 'radius', 50, 'frequency', 0.05)
        
        % Environment reference
        environment
        
        % Time tracker
        time = 0
        
        % Vehicle following properties
        leadVehicle = []      % Reference to lead vehicle (if following)
        followDistance = 15   % Distance to maintain from lead vehicle
    end
    
    methods
        function obj = Vehicle(environment)
            % Constructor
            obj.environment = environment;
            
            % Initialize position at one edge of the environment
            bounds = environment.getBounds();
            obj.position = [bounds(1,1) + 10, 0, 0];
            
            % Set orientation based on velocity
            obj.orientation = atan2(obj.velocity(2), obj.velocity(1));
        end
        
        function update(obj, dt)
            % Update vehicle state
            
            % Update time
            obj.time = obj.time + dt;
            
            % If following another vehicle
            if ~isempty(obj.leadVehicle)
                obj.updateFollowing(dt);
            else
                % Update position based on path type
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
            obj.orientation = atan2(obj.velocity(2), obj.velocity(1));
            
            % Apply environmental constraints
            obj.applyEnvironmentalConstraints();
        end
        
        function updateFollowing(obj, dt)
            % Update position when following another vehicle
            
            % Get lead vehicle position
            leadPos = obj.leadVehicle.position;
            leadVel = obj.leadVehicle.velocity;
            leadDir = leadVel / (norm(leadVel) + 1e-10); % Avoid division by zero
            
            % Calculate desired position (behind lead vehicle)
            desiredPos = leadPos - leadDir * obj.followDistance;
            
            % Calculate position error
            posError = desiredPos - obj.position;
            
            % PD controller for following
            kp = 1.0;  % Proportional gain
            kd = 2.0;  % Derivative gain
            
            % Calculate desired velocity
            desiredVel = kp * posError + leadVel;
            
            % Limit velocity magnitude
            maxSpeed = 10;
            velMagnitude = norm(desiredVel);
            if velMagnitude > maxSpeed
                desiredVel = desiredVel * (maxSpeed / velMagnitude);
            end
            
            % Update velocity and position
            obj.velocity = desiredVel;
            obj.position = obj.position + obj.velocity * dt;
        end
        
        function updateLinearPath(obj, dt)
            % Update position for linear path
            
            % Update position
            obj.position = obj.position + obj.velocity * dt;
            
            % Check if vehicle reaches environment boundary
            bounds = obj.environment.getBounds();
            if obj.position(1) <= bounds(1,1) || obj.position(1) >= bounds(1,2)
                obj.velocity(1) = -obj.velocity(1);
            end
            if obj.position(2) <= bounds(2,1) || obj.position(2) >= bounds(2,2)
                obj.velocity(2) = -obj.velocity(2);
            end
        end
        
        function updateCircularPath(obj, dt)
            % Update position for circular path
            
            % Calculate angular velocity
            angularVelocity = norm(obj.velocity) / obj.pathParams.radius;
            
            % Calculate angle
            angle = obj.time * angularVelocity;
            
            % Calculate new position
            center = obj.pathParams.center;
            obj.position(1) = center(1) + obj.pathParams.radius * cos(angle);
            obj.position(2) = center(2) + obj.pathParams.radius * sin(angle);
            
            % Calculate new velocity (tangential)
            obj.velocity(1) = -obj.pathParams.radius * angularVelocity * sin(angle);
            obj.velocity(2) = obj.pathParams.radius * angularVelocity * cos(angle);
        end
        
        function updateSinusoidalPath(obj, dt)
            % Update position for sinusoidal path
            
            % Calculate forward movement
            forwardSpeed = norm(obj.velocity);
            forwardDir = [cos(obj.orientation), sin(obj.orientation), 0];
            
            % Calculate lateral oscillation
            lateralDir = [-sin(obj.orientation), cos(obj.orientation), 0];
            lateralAmplitude = 20;
            lateralFreq = obj.pathParams.frequency;
            lateralOffset = lateralAmplitude * sin(obj.time * lateralFreq * 2 * pi);
            
            % Calculate new position
            basePos = obj.position + forwardDir * forwardSpeed * dt;
            targetLateralPos = obj.pathParams.center + lateralDir * lateralOffset;
            
            % Smoothly adjust lateral position
            lateralError = targetLateralPos - obj.position;
            lateralCorrection = lateralError * 0.1;
            
            % Update position
            obj.position = basePos + lateralCorrection;
            
            % Update velocity
            obj.velocity = forwardDir * forwardSpeed + lateralCorrection / dt;
        end
        
        function applyEnvironmentalConstraints(obj)
            % Apply environmental constraints
            
            % Boundary constraints
            bounds = obj.environment.getBounds();
            for i = 1:2  % Only constrain x and y
                if obj.position(i) < bounds(i, 1) + obj.length/2
                    obj.position(i) = bounds(i, 1) + obj.length/2;
                    if i == 1
                        obj.velocity(i) = abs(obj.velocity(i));
                    else
                        obj.velocity(i) = abs(obj.velocity(i));
                    end
                elseif obj.position(i) > bounds(i, 2) - obj.length/2
                    obj.position(i) = bounds(i, 2) - obj.length/2;
                    if i == 1
                        obj.velocity(i) = -abs(obj.velocity(i));
                    else
                        obj.velocity(i) = -abs(obj.velocity(i));
                    end
                end
            end
        end
        
        function padPos = getLandingPadPosition(obj)
            % Get landing pad position in world coordinates
            
            % Calculate rotation matrix
            R = [cos(obj.orientation), -sin(obj.orientation), 0;
                 sin(obj.orientation), cos(obj.orientation), 0;
                 0, 0, 1];
            
            % Transform landing pad offset to world coordinates
            padPos = obj.position + (R * obj.landingPadOffset')';
        end
        
        function drawVehicle(obj, ax)
            % Draw vehicle in the given axes
            
            % Calculate corners of vehicle body
            R = [cos(obj.orientation), -sin(obj.orientation), 0;
                 sin(obj.orientation), cos(obj.orientation), 0;
                 0, 0, 1];
            
            halfLength = obj.length / 2;
            halfWidth = obj.width / 2;
            
            corners = [
                halfLength, halfWidth, 0;
                halfLength, -halfWidth, 0;
                -halfLength, -halfWidth, 0;
                -halfLength, halfWidth, 0
            ];
            
            % Transform corners to world coordinates
            worldCorners = zeros(size(corners));
            for i = 1:size(corners, 1)
                worldCorners(i, :) = obj.position + (R * corners(i, :)')';
            end
            
            % Draw vehicle body
            X = [worldCorners(1:4, 1); worldCorners(1, 1)];
            Y = [worldCorners(1:4, 2); worldCorners(1, 2)];
            Z = [worldCorners(1:4, 3); worldCorners(1, 3)];
            
            % Bottom face
            fill3(ax, X, Y, Z, [0.3, 0.3, 0.3], 'EdgeColor', 'k');
            
            % Top face
            topZ = Z + obj.height;
            fill3(ax, X, Y, topZ, [0.3, 0.3, 0.3], 'EdgeColor', 'k');
            
            % Connect top and bottom faces
            for i = 1:4
                j = mod(i, 4) + 1;
                line(ax, [X(i), X(i)], [Y(i), Y(i)], [Z(i), topZ(i)], 'Color', 'k');
                line(ax, [X(i), X(j)], [Y(i), Y(j)], [topZ(i), topZ(j)], 'Color', 'k');
            end
            
            % Draw landing pad
            padPos = obj.getLandingPadPosition();
            padHalfSize = obj.landingPadSize / 2;
            
            padCorners = [
                padHalfSize, padHalfSize, 0;
                padHalfSize, -padHalfSize, 0;
                -padHalfSize, -padHalfSize, 0;
                -padHalfSize, padHalfSize, 0
            ];
            
            % Transform pad corners to world coordinates
            worldPadCorners = zeros(size(padCorners));
            for i = 1:size(padCorners, 1)
                worldPadCorners(i, :) = padPos + (R * padCorners(i, :)')';
            end
            
            % Draw landing pad
            X = [worldPadCorners(1:4, 1); worldPadCorners(1, 1)];
            Y = [worldPadCorners(1:4, 2); worldPadCorners(1, 2)];
            Z = [worldPadCorners(1:4, 3); worldPadCorners(1, 3)];
            
            fill3(ax, X, Y, Z, [0.2, 0.8, 0.2], 'EdgeColor', 'k');
            
            % Draw ArUco marker
            arUcoHalfSize = obj.arUcoSize / 2;
            arUcoCorners = [
                arUcoHalfSize, arUcoHalfSize, 0.01;
                arUcoHalfSize, -arUcoHalfSize, 0.01;
                -arUcoHalfSize, -arUcoHalfSize, 0.01;
                -arUcoHalfSize, arUcoHalfSize, 0.01
            ];
            
            % Transform ArUco corners to world coordinates
            worldArUcoCorners = zeros(size(arUcoCorners));
            for i = 1:size(arUcoCorners, 1)
                worldArUcoCorners(i, :) = padPos + (R * arUcoCorners(i, :)')';
            end
            
            % Draw ArUco marker background
            X = [worldArUcoCorners(1:4, 1); worldArUcoCorners(1, 1)];
            Y = [worldArUcoCorners(1:4, 2); worldArUcoCorners(1, 2)];
            Z = [worldArUcoCorners(1:4, 3); worldArUcoCorners(1, 3)];
            
            fill3(ax, X, Y, Z, [1, 1, 1], 'EdgeColor', 'k');
            
            % Draw ArUco marker pattern based on ID
            obj.drawArUcoPattern(ax, padPos, R);
            
            % Draw ArUco ID text
            textPos = padPos + (R * [0, 0, 0.05]')';
            text(ax, textPos(1), textPos(2), textPos(3), ...
                 ['ID: ', num2str(obj.arUcoID)], ...
                 'HorizontalAlignment', 'center', 'Color', 'r', 'FontWeight', 'bold');
        end
        
        function drawArUcoPattern(obj, ax, padPos, R)
            % Draw ArUco marker pattern based on ID
            
            % Different patterns for different IDs
            switch obj.arUcoID
                case 1
                    pattern = [
                        1 0 1 0 1;
                        0 1 0 1 0;
                        1 0 1 0 1;
                        0 1 0 1 0;
                        1 0 1 0 1
                    ];
                case 2
                    pattern = [
                        1 1 0 1 1;
                        1 0 1 0 1;
                        0 1 0 1 0;
                        1 0 1 0 1;
                        1 1 0 1 1
                    ];
                case 3
                    pattern = [
                        0 0 0 0 0;
                        0 1 1 1 0;
                        0 1 0 1 0;
                        0 1 1 1 0;
                        0 0 0 0 0
                    ];
                otherwise
                    pattern = eye(5);
            end
            
            % Draw pattern
            cellSize = obj.arUcoSize / 5;
            
            for i = 1:5
                for j = 1:5
                    if pattern(i, j) == 1
                        % Calculate cell position
                        cellX = (j - 3) * cellSize;
                        cellY = (i - 3) * cellSize;
                        
                        cellCorners = [
                            cellX + cellSize/2, cellY + cellSize/2, 0.02;
                            cellX + cellSize/2, cellY - cellSize/2, 0.02;
                            cellX - cellSize/2, cellY - cellSize/2, 0.02;
                            cellX - cellSize/2, cellY + cellSize/2, 0.02
                        ];
                        
                        % Transform cell corners to world coordinates
                        worldCellCorners = zeros(size(cellCorners));
                        for k = 1:size(cellCorners, 1)
                            worldCellCorners(k, :) = padPos + (R * cellCorners(k, :)')';
                        end
                        
                        % Draw cell
                        X = [worldCellCorners(1:4, 1); worldCellCorners(1, 1)];
                        Y = [worldCellCorners(1:4, 2); worldCellCorners(1, 2)];
                        Z = [worldCellCorners(1:4, 3); worldCellCorners(1, 3)];
                        
                        fill3(ax, X, Y, Z, [0, 0, 0], 'EdgeColor', 'none');
                    end
                end
            end
        end
    end
end
