classdef Security < handle
    % Security - Enhanced class for A3 with alternate key switching
    
    properties
        droneID = 'DRONE_001' % Unique drone identifier
        vehicleSecrets % Pre-shared primary secrets {vehicleID: secret}
        alternateSecrets % Pre-shared alternate secrets {vehicleID: secret}
        sessionKey = '' % Session key after association
        associationState = 'idle' % 'idle', 'initiated', 'confirmed', 'failed'
        isAuthorized = false % Tracks authorization status
        authAttempts = 0 % Tracks authentication attempts
        maxAuthAttempts = 3 % Maximum authentication attempts
        handshakeLatency = 0 % Simulated latency for handshake (seconds)
        associationFailures = 0 % Tracks failed association attempts
        lastMessageHMAC = '' % Store HMAC of last message
        spoofingDetected = false % Flag for spoofing detection
        spoofingAttempts = 0 % Track spoofing attempts
        usingAlternateKey = false % Track if alternate key is active
    end
    
    methods
        function obj = Security()
            % Constructor: Initialize primary and alternate secrets
            obj.vehicleSecrets = containers.Map;
            obj.alternateSecrets = containers.Map;
            for i = 1:3
                vehicleID = sprintf('VEHICLE_%d', i);
                obj.vehicleSecrets(vehicleID) = sprintf('SECRET_%s_%s_%s', obj.droneID, vehicleID, num2str(randi(10000)));
                obj.alternateSecrets(vehicleID) = sprintf('ALT_SECRET_%s_%s_%s', obj.droneID, vehicleID, num2str(randi(10000)));
            end
        end
        
        function success = authorizeLanding(obj, vehicle)
            % Check if drone is authorized to land
            if obj.isAuthorized
                success = true;
                return;
            end
            vehicleID = sprintf('VEHICLE_%d', vehicle.arUcoID);
            missionID = 'MISSION_001'; % Simulated mission ID
            authorizedMissions = {'MISSION_001', 'MISSION_002'};
            success = ismember(missionID, authorizedMissions);
            if success
                obj.isAuthorized = true;
                fprintf('Authorization granted for %s\n', vehicleID);
            else
                fprintf('Authorization failed for %s\n', vehicleID);
            end
        end
        
        function success = initiateAssociation(obj, vehicle)
            % Step 1: Initiate association with HMAC authentication
            vehicleID = sprintf('VEHICLE_%d', vehicle.arUcoID);
            if strcmp(obj.associationState, 'idle')
                obj.handshakeLatency = 0.1 * rand(); % Simulated network delay (0-0.1s)
                pause(obj.handshakeLatency); % Simulate processing time
                
                % Generate message and HMAC
                message = sprintf('%s|%s|INIT|%s', obj.droneID, vehicleID, num2str(obj.handshakeLatency));
                secret = obj.getActiveSecret(vehicleID);
                obj.lastMessageHMAC = obj.computeHMAC(message, secret);
                
                % Log message and HMAC
                fprintf('Initiating association with %s\n', vehicleID);
                fprintf('Message: %s\nHMAC: %s\n', message, obj.lastMessageHMAC);
                
                % Simulate receiving and verifying vehicle's response
                success = obj.verifyVehicleResponse(vehicleID, message, obj.lastMessageHMAC);
                
                if success
                    obj.associationState = 'initiated';
                    fprintf('Association initiated with %s (latency: %.3fs)\n', vehicleID, obj.handshakeLatency);
                else
                    obj.associationState = 'failed';
                    obj.associationFailures = obj.associationFailures + 1;
                    fprintf('Spoofing attempt detected: Invalid HMAC for %s\n', vehicleID);
                    obj.spoofingDetected = true;
                    obj.spoofingAttempts = obj.spoofingAttempts + 1;
                    % Switch to alternate key
                    if ~obj.usingAlternateKey
                        obj.usingAlternateKey = true;
                        fprintf('Switching to alternate key for %s\n', vehicleID);
                    end
                end
            else
                fprintf('Association initiation failed for %s: Invalid state (%s)\n', vehicleID, obj.associationState);
                success = false;
            end
        end
        
        function success = confirmAssociation(obj, vehicle)
            % Step 2: Confirm association with HMAC authentication
            vehicleID = sprintf('VEHICLE_%d', vehicle.arUcoID);
            if strcmp(obj.associationState, 'initiated')
                % Generate confirmation message and HMAC
                message = sprintf('%s|%s|CONFIRM|%s', obj.droneID, vehicleID, num2str(obj.handshakeLatency));
                secret = obj.getActiveSecret(vehicleID);
                obj.lastMessageHMAC = obj.computeHMAC(message, secret);
                
                % Log message and HMAC
                fprintf('Confirming association with %s\n', vehicleID);
                fprintf('Message: %s\nHMAC: %s\n', message, obj.lastMessageHMAC);
                
                % Simulate receiving and verifying vehicle's response
                success = obj.verifyVehicleResponse(vehicleID, message, obj.lastMessageHMAC);
                
                if success
                    obj.handshakeLatency = obj.handshakeLatency + 0.05 * rand(); % Additional delay
                    obj.sessionKey = obj.computeHash([obj.droneID, vehicleID, obj.getActiveSecret(vehicleID)]);
                    obj.associationState = 'confirmed';
                    fprintf('Association established with %s, session key: %s (total latency: %.3fs)\n', ...
                            vehicleID, obj.sessionKey, obj.handshakeLatency);
                else
                    obj.associationState = 'failed';
                    obj.associationFailures = obj.associationFailures + 1;
                    fprintf('Spoofing attempt detected: Invalid HMAC for %s\n', vehicleID);
                    obj.spoofingDetected = true;
                    obj.spoofingAttempts = obj.spoofingAttempts + 1;
                    % Switch to alternate key
                    if ~obj.usingAlternateKey
                        obj.usingAlternateKey = true;
                        fprintf('Switching to alternate key for %s\n', vehicleID);
                    end
                end
            else
                fprintf('Association confirmation failed for %s: Invalid state (%s)\n', vehicleID, obj.associationState);
                success = false;
            end
        end
        
        function secret = getActiveSecret(obj, vehicleID)
            % Get the active secret (primary or alternate)
            if obj.usingAlternateKey
                secret = obj.alternateSecrets(vehicleID);
            else
                secret = obj.vehicleSecrets(vehicleID);
            end
        end
        
        function hmac = computeHMAC(obj, message, secret)
            % Simulate HMAC using SHA-256-like hash
            input = [message, secret];
            hmac = obj.computeHash(input);
            hmac = sprintf('HMAC_%s', hmac);
        end
        
        function success = verifyVehicleResponse(obj, vehicleID, message, expectedHMAC)
            % Simulate vehicle's response and verify HMAC
            secret = obj.getActiveSecret(vehicleID);
            
            % Simulate a spoofing attack with 40% probability
            if rand() < 0.4
                % Spoofed response with incorrect HMAC
                spoofedHMAC = obj.computeHMAC(message, 'WRONG_SECRET');
                success = strcmp(spoofedHMAC, expectedHMAC);
            else
                % Legitimate response
                receivedHMAC = obj.computeHMAC(message, secret);
                success = strcmp(receivedHMAC, expectedHMAC);
            end
        end
        
        function hash = computeHash(obj, input)
            % Simulate SHA-256-like hash
            inputStr = sprintf('%s', strjoin(cellstr(input), '|'));
            hashVal = 0;
            for i = 1:length(inputStr)
                hashVal = mod(hashVal * 31 + double(inputStr(i)), 2^32);
            end
            hash = sprintf('SHA256_%08x', hashVal);
        end
        
        function resetAssociation(obj)
            % Reset association state
            obj.associationState = 'idle';
            obj.sessionKey = '';
            obj.isAuthorized = false;
            obj.handshakeLatency = 0;
            obj.lastMessageHMAC = '';
            obj.spoofingDetected = false;
            obj.usingAlternateKey = false; % Reset to primary key
        end
        
        function metrics = getSecurityMetrics(obj)
            % Return security metrics for analysis
            metrics = struct(...
                'authAttempts', obj.authAttempts, ...
                'associationFailures', obj.associationFailures, ...
                'handshakeLatency', obj.handshakeLatency, ...
                'spoofingAttempts', obj.spoofingAttempts ...
            );
        end
    end
end
