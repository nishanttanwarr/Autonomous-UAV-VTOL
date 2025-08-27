classdef DroneSimulation < handle
    % DroneSimulation - Main class for the VTOL UAV landing simulation
    
    properties
        environment
        vehicles
        targetVehicleIndex = 0
        drone
        security % Security manager for A3
        timeStep = 0.05
        maxTime = 300
        currentTime = 0
        visualizer
        dashboard
        flightDashboard
        isAutonomous = true
        landingComplete = false
        targetArUcoID
        arUcoDetectionStatus = 'Not Detected' % For dashboard display
        assocRetryCount = 0 % Track association retries
        maxAssocRetries = 3 % Maximum association retries
        
        % Wind vector for the simulation environment
        wind_vector = [4.0, -2.0, 0];
        
        %%% LANDING: State machine property %%%
        drone_state = 'SEARCHING';
        
        %%% ADDED FOR BOUNDING BOX SCAN %%%
        search_waypoints = [];
        current_waypoint_index = 1;
        last_waypoint_gen_time = -10; % Force generation on first run
    end
    
    methods
        function obj = DroneSimulation()
            % Constructor
            obj.environment = Environment();
            
            % Create vehicles
            obj.vehicles = cell(1, 3);
            
            %%% SCENARIO CHANGE: Create a tight convoy on a straight road %%%
            
            % Lead Vehicle
            obj.vehicles{1} = Vehicle(obj.environment);
            obj.vehicles{1}.position = [0, -80, 0]; % Start at one end of the road
            obj.vehicles{1}.velocity = [0, 8, 0];   % Move along the Y-axis
            obj.vehicles{1}.arUcoID = 1;
            obj.vehicles{1}.autoProfile = true; % Leader can vary its speed
            obj.vehicles{1}.pathType = 'linear';
            
            % Follower Vehicle 1
            obj.vehicles{2} = Vehicle(obj.environment);
            obj.vehicles{2}.position = [0, -100, 0]; % Start behind the leader
            obj.vehicles{2}.arUcoID = 2;
            obj.vehicles{2}.leadVehicle = obj.vehicles{1}; % Set to follow vehicle 1
            obj.vehicles{2}.followDistance = 20; % Maintain 20m distance
            obj.vehicles{2}.autoProfile = false; % Disable random speed changes
            
            % Follower Vehicle 2
            obj.vehicles{3} = Vehicle(obj.environment);
            obj.vehicles{3}.position = [0, -120, 0]; % Start behind vehicle 2
            obj.vehicles{3}.arUcoID = 3;
            obj.vehicles{3}.leadVehicle = obj.vehicles{2}; % Set to follow vehicle 2
            obj.vehicles{3}.followDistance = 20; % Maintain 20m distance
            obj.vehicles{3}.autoProfile = false; % Disable random speed changes
            
            obj.drone = Drone(obj.environment);
            obj.drone.position = [20, -140, 20]; % Start behind and to the side of the convoy
            
            obj.security = Security();
            obj.visualizer = Visualizer(obj.environment, obj.vehicles, obj.drone);
            obj.dashboard = Dashboard(obj);
            obj.flightDashboard = FlightDashboard(obj);
            
            obj.promptForTargetArUcoID();
        end
        
        function promptForTargetArUcoID(obj)
            validInput = false;
            while ~validInput
                prompt = 'Enter the ArUco marker ID to land on (1, 2, or 3): ';
                val = input(prompt);
                if isempty(val)
                    obj.targetArUcoID = 1;
                    validInput = true;
                    fprintf('No input - defaulting to ArUco ID: %d\n', obj.targetArUcoID);
                elseif ismember(val, [1, 2, 3])
                    obj.targetArUcoID = val;
                    validInput = true;
                    fprintf('Target set to vehicle with ArUco marker ID: %d\n', obj.targetArUcoID);
                else
                    fprintf('Invalid input. Please enter 1, 2, or 3.\n');
                end
            end
        end
        
        function run(obj)
            % Run simulation
            fig = figure('Name', 'VTOL UAV Landing Simulation', ...
                         'KeyPressFcn', @obj.keyPressCallback, ...
                         'Position', [50, 50, 1200, 800]);
            obj.visualizer.setupFigure(fig);
            obj.dashboard.setupDashboard(fig);
            obj.flightDashboard.setupFlightDashboard(fig);
            
            % Main loop
            while obj.currentTime < obj.maxTime && ~obj.landingComplete
                if ~ishandle(fig)
                    break;
                end
                
                if ismethod(obj.environment, 'update')
                    obj.environment.update(obj.currentTime);
                end
                
                for i = 1:length(obj.vehicles)
                    try
                        obj.vehicles{i}.update(obj.timeStep);
                    catch ME
                        warning('Vehicle %d update failed: %s', i, ME.message);
                    end
                end
                
                if obj.isAutonomous
                    obj.autonomousControl();
                end
                
                obj.drone.update(obj.timeStep, obj.wind_vector);
                
                obj.checkLanding();
                
                try
                    obj.visualizer.update();
                catch ME
                    warning('Visualizer update failed: %s', ME.message);
                end
                try
                    obj.dashboard.update();
                catch ME
                    warning('Dashboard update failed: %s', ME.message);
                end
                try
                    obj.flightDashboard.update();
                catch ME
                    warning('FlightDashboard update failed: %s', ME.message);
                end
                
                obj.currentTime = obj.currentTime + obj.timeStep;
                
                pause(obj.timeStep);
            end
            
            obj.displayResults();
        end
        
        function autonomousControl(obj)
            % State-machine based autonomous control with DEBUG MESSAGES.
            
            if obj.targetVehicleIndex == 0
                obj.drone_state = 'SEARCHING';
            end
            targetVehicle = [];
            if obj.targetVehicleIndex > 0
                targetVehicle = obj.vehicles{obj.targetVehicleIndex};
            end
            % --- Main State Machine ---
            switch obj.drone_state
                case 'SEARCHING'
                    fprintf('State: SEARCHING | Time: %.2f\n', obj.currentTime);
                    obj.searchForTargetVehicle();
                    if obj.targetVehicleIndex > 0
                        disp('Target vehicle found! Switching to APPROACH state.');
                        obj.drone_state = 'APPROACH';
                    end
                case 'APPROACH'
                    landingPadPos = targetVehicle.getLandingPadPosition();
                    vehicleDir = targetVehicle.velocity / (norm(targetVehicle.velocity) + 1e-9);
                    stagingPoint = landingPadPos - vehicleDir * 15 + [0, 0, 10];
                    obj.drone.setTargetPosition(stagingPoint);
                    obj.drone.setTargetVelocity(targetVehicle.velocity);
                    distToStaging = norm(obj.drone.position - stagingPoint);
                    fprintf('State: APPROACH | Time: %.2f | Distance to Staging Point: %.2f m\n', obj.currentTime, distToStaging);
                    
                    if distToStaging < 2.0 % Tolerance of 2 meters
                        disp('Staging point reached. Switching to ALIGN state.');
                        obj.drone_state = 'ALIGN';
                    end
                case 'ALIGN'
                    landingPadPos = targetVehicle.getLandingPadPosition();
                    hoverPoint = landingPadPos + [0, 0, 5];
                    obj.drone.setTargetPosition(hoverPoint);
                    obj.drone.setTargetVelocity(targetVehicle.velocity);
                    if ~obj.drone.arUcoAuthenticated
                        obj.authenticateArUco();
                    else
                        obj.performA3Process(targetVehicle);
                    end
                    
                    distToHover = norm(obj.drone.position(1:2) - hoverPoint(1:2));
                    isAuth = obj.drone.arUcoAuthenticated;
                    assocState = obj.security.associationState;
                    isReadyToLand = isAuth && strcmp(assocState, 'confirmed');
                    fprintf('State: ALIGN | Time: %.2f | Dist to Hover: %.2f m | Authenticated: %d | Association: %s\n', ...
                        obj.currentTime, distToHover, isAuth, assocState);
                    if distToHover < 0.5 && isReadyToLand
                        disp('Aligned over pad and authenticated. Switching to LANDING state.');
                        obj.drone_state = 'LANDING';
                    end
                case 'LANDING'
                    landingPadPos = targetVehicle.getLandingPadPosition();
                    obj.drone.setTargetPosition(landingPadPos); 
                    obj.drone.setTargetVelocity(targetVehicle.velocity);
                    obj.drone.setDescentRate(1.0);
                    distToPad = norm(obj.drone.position - landingPadPos);
                    fprintf('State: LANDING | Time: %.2f | Distance to Pad: %.2f m\n', obj.currentTime, distToPad);
            end
        end
        
        function searchForTargetVehicle(obj)
            searchHeight = 30; % Altitude for searching
            
            % --- 1. Regenerate waypoints every 5 seconds to track the group ---
            if (obj.currentTime - obj.last_waypoint_gen_time) > 5
                disp('Regenerating search pattern based on vehicle positions...');
                
                % Find the bounding box of all vehicles
                min_x = inf; max_x = -inf; min_y = inf; max_y = -inf;
                for i = 1:length(obj.vehicles)
                    pos = obj.vehicles{i}.position;
                    min_x = min(min_x, pos(1));
                    max_x = max(max_x, pos(1));
                    min_y = min(min_y, pos(2));
                    max_y = max(max_y, pos(2));
                end
                
                % Add a buffer to the box
                buffer = 15; % meters
                min_x = min_x - buffer; max_x = max_x + buffer;
                min_y = min_y - buffer; max_y = max_y + buffer;
                
                % Generate lawnmower pattern waypoints
                obj.search_waypoints = [];
                leg_spacing = 20; % 20 meters between scan lines
                direction = 1; % 1 for forward, -1 for backward
                
                for x = min_x:leg_spacing:max_x
                    if direction == 1
                        obj.search_waypoints = [obj.search_waypoints; x, min_y, searchHeight];
                        obj.search_waypoints = [obj.search_waypoints; x, max_y, searchHeight];
                    else
                        obj.search_waypoints = [obj.search_waypoints; x, max_y, searchHeight];
                        obj.search_waypoints = [obj.search_waypoints; x, min_y, searchHeight];
                    end
                    direction = direction * -1;
                end
                
                obj.current_waypoint_index = 1;
                obj.last_waypoint_gen_time = obj.currentTime;
            end
            
            % --- 2. Fly to the current waypoint ---
            if isempty(obj.search_waypoints)
                return; % Wait for waypoints to be generated
            end
            
            target_wp = obj.search_waypoints(obj.current_waypoint_index, :);
            obj.drone.setTargetPosition(target_wp);
            obj.drone.setTargetYaw(NaN); % Let drone point towards waypoint
            
            % Check if waypoint is reached
            dist_to_wp = norm(obj.drone.position - target_wp);
            if dist_to_wp < 3.0 % Waypoint reached tolerance
                obj.current_waypoint_index = obj.current_waypoint_index + 1;
                % If we finish the pattern, loop back and regenerate
                if obj.current_waypoint_index > size(obj.search_waypoints, 1)
                    obj.last_waypoint_gen_time = -10; % Force regeneration
                end
            end
            
            % --- 3. Check for detection ---
            for i = 1:length(obj.vehicles)
                if obj.simulateArUcoDetection(obj.vehicles{i}) == obj.targetArUcoID
                    obj.targetVehicleIndex = i;
                    fprintf('Target vehicle %d found! Switching to APPROACH.\n', i);
                    return;
                end
            end
        end
        
        function detectedID = simulateArUcoDetection(obj, vehicle)
            padPos = vehicle.getLandingPadPosition();
            rel = padPos - obj.drone.position;
            horizDist = norm(rel(1:2));
            
            % Correctly calculate vertical distance (altitude above the pad)
            vert = obj.drone.position(3) - padPos(3);
            
            detectionRange = 25;
            minAltitudeForDetection = 1;
            maxAltitudeForDetection = 60;
            
            detectedID = 0;
            % Check if drone is within the vertical and horizontal detection range
            if horizDist <= detectionRange && vert > minAltitudeForDetection && vert < maxAltitudeForDetection
                % Check if the vehicle is within the camera's field of view
                droneYaw = obj.drone.orientation(3);
                forwardVec = [cos(droneYaw), sin(droneYaw), 0];
                
                % Ensure vecToTarget is a 3D vector for the dot product
                vecToTarget_2d = rel(1:2) / max(horizDist, 1e-9);
                vecToTarget = [vecToTarget_2d, 0];
                
                % Angle between drone's forward direction and the target
                angle_to_target = acos(dot(forwardVec, vecToTarget));
                
                if abs(angle_to_target) < (pi/4) % Camera FOV is +/- 45 degrees
                    % Simulate probabilistic detection
                    pDetect = max(0.2, 1 - (horizDist / detectionRange));
                    if rand() < pDetect
                        detectedID = vehicle.arUcoID;
                    end
                end
            end
        end
        
        function detectedID = getDetectedArUcoID(obj)
            detectedID = 0;
            for i = 1:length(obj.vehicles)
                vehicle = obj.vehicles{i};
                tmp = obj.simulateArUcoDetection(vehicle);
                if tmp > 0
                    detectedID = tmp;
                    return;
                end
            end
        end
        
        function authenticateArUco(obj)
            if obj.targetVehicleIndex <= 0 || obj.targetVehicleIndex > length(obj.vehicles)
                return;
            end
            targetVehicle = obj.vehicles{obj.targetVehicleIndex};
            rel = obj.drone.position - targetVehicle.getLandingPadPosition();
            horizDist = norm(rel(1:2));
            vert = rel(3);
            
            if horizDist < 5 && vert < 10 && vert > 0
                obj.security.authAttempts = obj.security.authAttempts + 1;
                if rand() > 0.2 && obj.security.authAttempts <= obj.security.maxAuthAttempts
                    if targetVehicle.arUcoID == obj.targetArUcoID
                        obj.drone.arUcoAuthenticated = true;
                        obj.drone.authenticatedArUcoID = obj.targetArUcoID;
                        obj.arUcoDetectionStatus = 'Authenticated';
                        disp('ArUco marker authenticated successfully!');
                    else
                        disp('ArUco marker ID does not match target! Aborting landing.');
                    end
                else
                    disp('ArUco marker authentication failed. Retrying...');
                    if obj.security.authAttempts >= obj.security.maxAuthAttempts
                        disp('Maximum authentication attempts reached. Aborting landing.');
                        obj.landingComplete = true;
                    end
                end
            end
        end
        
        function performA3Process(obj, vehicle)
            if isempty(vehicle) || ~obj.drone.arUcoAuthenticated
                return;
            end
            
            if obj.drone.arUcoAuthenticated && ~strcmp(obj.security.associationState, 'confirmed')
                authzSuccess = obj.security.authorizeLanding(vehicle);
                if authzSuccess
                    switch obj.security.associationState
                        case 'idle'
                            if obj.security.initiateAssociation(vehicle)
                                obj.assocRetryCount = 0;
                                if obj.security.spoofingDetected
                                    disp('Spoofing attempt detected during association initiation!');
                                    if obj.security.usingAlternateKey
                                        obj.dashboard.addStatusLog(sprintf('Switched to alternate key for VEHICLE_%d', vehicle.arUcoID));
                                    end
                                end
                            end
                        case 'initiated'
                            if obj.security.confirmAssociation(vehicle)
                                obj.assocRetryCount = 0;
                                if obj.security.spoofingDetected
                                    disp('Spoofing attempt detected during association confirmation!');
                                    if obj.security.usingAlternateKey
                                        obj.dashboard.addStatusLog(sprintf('Switched to alternate key for VEHICLE_%d', vehicle.arUcoID));
                                    end
                                end
                            else
                                obj.assocRetryCount = obj.assocRetryCount + 1;
                                if obj.assocRetryCount >= obj.maxAssocRetries
                                    disp('Maximum association retries reached. Aborting landing.');
                                    obj.landingComplete = true;
                                else
                                    obj.security.resetAssociation();
                                    disp('Retrying association...');
                                end
                            end
                        case 'failed'
                            obj.assocRetryCount = obj.assocRetryCount + 1;
                            if obj.assocRetryCount >= obj.maxAssocRetries
                                disp('Maximum association retries reached. Aborting landing.');
                                obj.landingComplete = true;
                            else
                                obj.security.resetAssociation();
                                disp('Retrying association...');
                            end
                    end
                end
            end
        end
        
        function checkLanding(obj)
            if obj.targetVehicleIndex == 0
                return;
            end
            dronePos = obj.drone.position;
            targetVehicle = obj.vehicles{obj.targetVehicleIndex};
            landingPadPos = targetVehicle.getLandingPadPosition();
            distance = norm(dronePos - landingPadPos);
            velDiff = norm(obj.drone.velocity(1:2) - targetVehicle.velocity(1:2));
            if distance < 0.5 && velDiff < 0.5 && strcmp(obj.security.associationState, 'confirmed')
                obj.landingComplete = true;
                disp('Landing complete!');
            end
        end
        
        function keyPressCallback(obj, ~, event)
            switch event.Key
                case 'a'
                    obj.isAutonomous = ~obj.isAutonomous;
                    if obj.isAutonomous
                        disp('Switched to autonomous control');
                    else
                        disp('Switched to manual control');
                    end
                case 'v'
                    obj.visualizer.toggleView();
                    disp(['Switched to ', obj.visualizer.currentView, ' view']);
                case 'd'
                    obj.dashboard.toggleView();
                case 'f'
                    if isprop(obj, 'flightDashboard') && isvalid(obj.flightDashboard)
                        obj.flightDashboard.toggleView();
                        disp('Toggled flight dashboard visibility');
                    end
                case 'g'
                    if ~obj.isAutonomous
                        x = input('Enter X coordinate: ');
                        y = input('Enter Y coordinate: ');
                        z = input('Enter Z coordinate: ');
                        obj.drone.setTargetPosition([x, y, z]);
                        disp(['Setting target position to [', num2str(x), ', ', num2str(y), ', ', num2str(z), ']']);
                    end
                case 'uparrow'
                    if ~obj.isAutonomous
                        obj.drone.manualControl([1, 0, 0]);
                    end
                case 'downarrow'
                    if ~obj.isAutonomous
                        obj.drone.manualControl([-1, 0, 0]);
                    end
                case 'leftarrow'
                    if ~obj.isAutonomous
                        obj.drone.manualControl([0, -1, 0]);
                    end
                case 'rightarrow'
                    if ~obj.isAutonomous
                        obj.drone.manualControl([0, 1, 0]);
                    end
                case 'pageup'
                    if ~obj.isAutonomous
                        obj.drone.manualControl([0, 0, 1]);
                    end
                case 'pagedown'
                    if ~obj.isAutonomous
                        obj.drone.manualControl([0, 0, -1]);
                    end
            end
        end
        
        function displayResults(obj)
            % Check if the figure window still exists before trying to show a message box.
            if isprop(obj.visualizer, 'fig') && isvalid(obj.visualizer.fig)
                metrics = obj.security.getSecurityMetrics();
                if obj.landingComplete && strcmp(obj.security.associationState, 'confirmed')
                    msgbox(sprintf(['Landing completed successfully on vehicle with ArUco ID %d at time %.2f seconds!\n', ...
                                    'Authentication Attempts: %d\nAssociation Failures: %d\nHandshake Latency: %.3fs\nSpoofing Attempts: %d'], ...
                                   obj.targetArUcoID, obj.currentTime, metrics.authAttempts, ...
                                   metrics.associationFailures, metrics.handshakeLatency, metrics.spoofingAttempts), 'Success');
                else
                    msgbox(sprintf(['Simulation ended at time %.2f seconds without successful landing.\n', ...
                                    'Authentication Attempts: %d\nAssociation Failures: %d\nHandshake Latency: %.3fs\nSpoofing Attempts: %d'], ...
                                   obj.currentTime, metrics.authAttempts, metrics.associationFailures, ...
                                   metrics.handshakeLatency, metrics.spoofingAttempts), 'Simulation End');
                end
            else
                disp('Simulation window was closed. Skipping final message box.');
            end
        end
    end
end
