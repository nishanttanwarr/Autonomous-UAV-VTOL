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
    end
    
    methods
        function obj = DroneSimulation()
            % Constructor
            obj.environment = Environment();
            obj.vehicles = cell(1, 3);
            obj.vehicles{1} = Vehicle(obj.environment);
            obj.vehicles{1}.position = [0, 0, 0];
            obj.vehicles{1}.arUcoID = 1;
            for i = 2:3
                obj.vehicles{i} = Vehicle(obj.environment);
                obj.vehicles{i}.position = [-(i-1)*15, 0, 0];
                obj.vehicles{i}.arUcoID = i;
            end
            obj.drone = Drone(obj.environment);
            obj.security = Security();
            obj.visualizer = Visualizer(obj.environment, obj.vehicles, obj.drone);
            obj.dashboard = Dashboard(obj);
            obj.flightDashboard = FlightDashboard(obj);
            obj.promptForTargetArUcoID();
        end
        
        function promptForTargetArUcoID(obj)
            % Prompt for target ArUco ID
            validInput = false;
            while ~validInput
                prompt = 'Enter the ArUco marker ID to land on (1, 2, or 3): ';
                obj.targetArUcoID = input(prompt);
                if ismember(obj.targetArUcoID, [1, 2, 3])
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
            while obj.currentTime < obj.maxTime && ~obj.landingComplete
                for i = 1:length(obj.vehicles)
                    obj.vehicles{i}.update(obj.timeStep);
                end
                if obj.isAutonomous
                    obj.autonomousControl();
                end
                obj.drone.update(obj.timeStep);
                obj.checkLanding();
                obj.visualizer.update();
                obj.dashboard.update();
                obj.flightDashboard.update();
                obj.currentTime = obj.currentTime + obj.timeStep;
                pause(obj.timeStep);
                if ~ishandle(fig)
                    break;
                end
            end
            obj.displayResults();
        end
        
        function autonomousControl(obj)
            % Autonomous control with A3
            dronePos = obj.drone.position;
            if obj.targetVehicleIndex == 0
                obj.searchForTargetVehicle();
            else
                targetVehicle = obj.vehicles{obj.targetVehicleIndex};
                landingPadPos = targetVehicle.getLandingPadPosition();
                distance = norm(dronePos - landingPadPos);
                if distance > 20
                    obj.drone.setTargetPosition(landingPadPos + [0, 0, 15]);
                    obj.drone.setTargetVelocity(targetVehicle.velocity);
                    obj.security.resetAssociation();
                    obj.assocRetryCount = 0;
                elseif distance > 5
                    obj.drone.setTargetPosition(landingPadPos + [0, 0, 4]);
                    obj.drone.setTargetVelocity(targetVehicle.velocity);
                    if distance < 10 && ~obj.drone.arUcoAuthenticated
                        obj.authenticateArUco();
                    end
                    if obj.drone.arUcoAuthenticated
                        obj.performA3Process(targetVehicle);
                    end
                elseif obj.drone.arUcoAuthenticated && strcmp(obj.security.associationState, 'confirmed')
                    descentRate = min(0.5, distance / 2);
                    obj.drone.setTargetPosition(landingPadPos);
                    obj.drone.setTargetVelocity(targetVehicle.velocity);
                    obj.drone.setDescentRate(descentRate);
                else
                    obj.drone.setTargetPosition(landingPadPos + [0, 0, 5]);
                    obj.drone.setTargetVelocity(targetVehicle.velocity);
                    if obj.drone.arUcoAuthenticated
                        obj.performA3Process(targetVehicle);
                    else
                        obj.authenticateArUco();
                    end
                end
            end
        end
        
        function searchForTargetVehicle(obj)
            % Search for target vehicle
            bounds = obj.environment.getBounds();
            centerX = (bounds(1,1) + bounds(1,2)) / 2;
            centerY = (bounds(2,1) + bounds(2,2)) / 2;
            searchHeight = 30;
            obj.drone.setTargetPosition([centerX, centerY, searchHeight]);
            obj.drone.setTargetVelocity([0, 0, 0]);
            for i = 1:length(obj.vehicles)
                vehicle = obj.vehicles{i};
                vehiclePos = vehicle.getLandingPadPosition();
                distance = norm(obj.drone.position - vehiclePos);
                relPos = obj.drone.position - vehiclePos;
                if distance < 40 && abs(atan2(relPos(2), relPos(1)) - obj.drone.orientation(3)) < pi/4
                    detectedID = obj.simulateArUcoDetection(vehicle);
                    if detectedID > 0
                        fprintf('Detected vehicle with ArUco ID: %d\n', detectedID);
                        obj.arUcoDetectionStatus = 'Detected';
                        if detectedID == obj.targetArUcoID
                            obj.targetVehicleIndex = i;
                            fprintf('Target vehicle found! Beginning approach.\n');
                            return;
                        end
                    else
                        obj.arUcoDetectionStatus = 'Not Detected';
                    end
                else
                    obj.arUcoDetectionStatus = 'Not Detected';
                end
            end
            if obj.targetVehicleIndex == 0
                radius = 40;
                angle = obj.currentTime * 0.2;
                searchX = centerX + radius * cos(angle);
                searchY = centerY + radius * sin(angle);
                obj.drone.setTargetPosition([searchX, searchY, searchHeight]);
            end
        end
        
        function detectedID = simulateArUcoDetection(obj, vehicle)
            % Simulate ArUco detection
            relPos = obj.drone.position - vehicle.getLandingPadPosition();
            if norm(relPos(1:2)) < 20 && relPos(3) < 40 && relPos(3) > 0
                detectedID = vehicle.arUcoID;
            else
                detectedID = 0;
            end
        end
        
        function detectedID = getDetectedArUcoID(obj)
            % Get the ArUco ID of the most recently detected vehicle
            detectedID = 0;
            for i = 1:length(obj.vehicles)
                vehicle = obj.vehicles{i};
                tempID = obj.simulateArUcoDetection(vehicle);
                if tempID > 0
                    detectedID = tempID;
                    break;
                end
            end
        end
        
        function authenticateArUco(obj)
            % Authenticate ArUco marker
            targetVehicle = obj.vehicles{obj.targetVehicleIndex};
            relPos = obj.drone.position - targetVehicle.getLandingPadPosition();
            if norm(relPos(1:2)) < 5 && relPos(3) < 10 && relPos(3) > 0
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
            % Perform A3 process with retry logic
            if obj.drone.arUcoAuthenticated && ~strcmp(obj.security.associationState, 'confirmed')
                authzSuccess = obj.security.authorizeLanding(vehicle);
                if authzSuccess
                    if strcmp(obj.security.associationState, 'idle')
                        if obj.security.initiateAssociation(vehicle)
                            obj.assocRetryCount = 0;
                            if obj.security.spoofingDetected
                                disp('Spoofing attempt detected during association initiation!');
                                if obj.security.usingAlternateKey
                                    obj.dashboard.addStatusLog(sprintf('Switched to alternate key for VEHICLE_%d', vehicle.arUcoID));
                                end
                            end
                        end
                    elseif strcmp(obj.security.associationState, 'initiated')
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
                    elseif strcmp(obj.security.associationState, 'failed')
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
            % Check landing completion
            if obj.targetVehicleIndex == 0
                return;
            end
            dronePos = obj.drone.position;
            targetVehicle = obj.vehicles{obj.targetVehicleIndex};
            landingPadPos = targetVehicle.getLandingPadPosition();
            distance = norm(dronePos - landingPadPos);
            if distance < 0.5 && norm(obj.drone.velocity - targetVehicle.velocity) < 0.5 && strcmp(obj.security.associationState, 'confirmed')
                obj.landingComplete = true;
                disp('Landing complete!');
            end
        end
        
        function keyPressCallback(obj, ~, event)
            % Handle keyboard input
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
                    obj.flightDashboard.toggleView();
                    disp('Toggled flight dashboard visibility');
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
            % Display results
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
        end
    end
end
