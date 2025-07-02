classdef Visualizer < handle
    % Visualizer - Class for visualizing the simulation
    
    properties
        % References to simulation objects
        environment
        vehicles
        drone
        
        % Visualization properties
        figure
        axes
        
        % View properties
        currentView = 'third-person'  % 'third-person', 'drone-pov', 'top-down'
        
        % Camera properties
        cameraPosition = [0, -50, 30]
        cameraTarget = [0, 0, 0]
        cameraUp = [0, 0, 1]
        
        % Trajectory visualization
        droneTrajectory = []
        vehicleTrajectories = {}
        maxTrajectoryPoints = 100
        
        % HUD elements
        hudAxes
        hudElements = struct()
    end
    
    methods
        function obj = Visualizer(environment, vehicles, drone)
            % Constructor
            obj.environment = environment;
            obj.vehicles = vehicles;
            obj.drone = drone;
            
            % Initialize vehicle trajectories
            obj.vehicleTrajectories = cell(1, length(vehicles));
            for i = 1:length(vehicles)
                obj.vehicleTrajectories{i} = [];
            end
        end
        
        function setupFigure(obj, fig)
            % Setup figure and axes
            obj.figure = fig;
            
            % Create main 3D axes (adjusted to fit above flight dashboard)
            obj.axes = axes('Parent', fig, 'Position', [0.01, 0.31, 0.58, 0.68]); % Top 70% of left 60%
            hold(obj.axes, 'on');
            grid(obj.axes, 'on');
            box(obj.axes, 'on');
            
            % Set initial view
            obj.updateView();
            
            % Create HUD axes (overlay)
            obj.hudAxes = axes('Parent', fig, 'Position', [0, 0, 1, 1], ...
                              'Color', 'none', 'XColor', 'none', 'YColor', 'none', ...
                              'ZColor', 'none', 'Box', 'off', 'Visible', 'off');
            hold(obj.hudAxes, 'on');
            axis(obj.hudAxes, [0 1 0 1]);
            
            % Setup HUD elements
            obj.setupHUD();
        end
        
        function setupHUD(obj)
            % Setup HUD elements
            
            % Status text
            obj.hudElements.statusText = text(obj.hudAxes, 0.05, 0.95, '', ...
                'FontSize', 10, 'Color', 'k', 'VerticalAlignment', 'top');
            
            % Altitude indicator
            obj.hudElements.altitudeText = text(obj.hudAxes, 0.05, 0.9, '', ...
                'FontSize', 10, 'Color', 'k', 'VerticalAlignment', 'top');
            
            % Speed indicator
            obj.hudElements.speedText = text(obj.hudAxes, 0.05, 0.85, '', ...
                'FontSize', 10, 'Color', 'k', 'VerticalAlignment', 'top');
            
            % Distance indicator
            obj.hudElements.distanceText = text(obj.hudAxes, 0.05, 0.8, '', ...
                'FontSize', 10, 'Color', 'k', 'VerticalAlignment', 'top');
            
            % ArUco status
            obj.hudElements.arUcoText = text(obj.hudAxes, 0.05, 0.75, '', ...
                'FontSize', 10, 'Color', 'k', 'VerticalAlignment', 'top');
            
            % View mode indicator
            obj.hudElements.viewText = text(obj.hudAxes, 0.8, 0.95, '', ...
                'FontSize', 10, 'Color', 'k', 'VerticalAlignment', 'top');
            
            % Controls help (updated to include flight dashboard toggle)
            obj.hudElements.helpText = text(obj.hudAxes, 0.05, 0.1, ...
                {'Controls:', ...
                 'A: Toggle autonomous/manual mode', ...
                 'V: Toggle view', ...
                 'D: Toggle simulation dashboard', ...
                 'F: Toggle flight dashboard', ...
                 'G: Go to coordinates (manual mode)', ...
                 'Arrow keys: Move drone (manual mode)', ...
                 'Page Up/Down: Adjust altitude (manual mode)'}, ...
                'FontSize', 8, 'Color', 'k', 'VerticalAlignment', 'top');
        end
        
        function update(obj)
            % Update visualization
            
            % Clear axes
            cla(obj.axes);
            
            % Draw environment
            obj.environment.drawEnvironment(obj.axes);
            
            % Draw vehicles
            for i = 1:length(obj.vehicles)
                obj.vehicles{i}.drawVehicle(obj.axes);
            end
            
            % Draw drone
            obj.drone.drawDrone(obj.axes);
            
            % Update trajectories
            obj.updateTrajectories();
            
            % Update view
            obj.updateView();
            
            % Update HUD
            obj.updateHUD();
            
            % Refresh display
            drawnow;
        end
        
        function updateTrajectories(obj)
            % Update trajectory visualization
            
            % Add current positions to trajectories
            obj.droneTrajectory = [obj.droneTrajectory; obj.drone.position];
            
            % Update vehicle trajectories
            for i = 1:length(obj.vehicles)
                obj.vehicleTrajectories{i} = [obj.vehicleTrajectories{i}; obj.vehicles{i}.getLandingPadPosition()];
            end
            
            % Limit number of points
            if size(obj.droneTrajectory, 1) > obj.maxTrajectoryPoints
                obj.droneTrajectory = obj.droneTrajectory(end-obj.maxTrajectoryPoints+1:end, :);
            end
            
            for i = 1:length(obj.vehicleTrajectories)
                if size(obj.vehicleTrajectories{i}, 1) > obj.maxTrajectoryPoints
                    obj.vehicleTrajectories{i} = obj.vehicleTrajectories{i}(end-obj.maxTrajectoryPoints+1:end, :);
                end
            end
            
            % Draw trajectories
            if size(obj.droneTrajectory, 1) > 1
                plot3(obj.axes, obj.droneTrajectory(:,1), obj.droneTrajectory(:,2), obj.droneTrajectory(:,3), ...
                    'b-', 'LineWidth', 1);
            end
            
            % Draw vehicle trajectories with different colors
            colors = {'r-', 'g-', 'm-'};
            for i = 1:length(obj.vehicleTrajectories)
                if size(obj.vehicleTrajectories{i}, 1) > 1
                    plot3(obj.axes, obj.vehicleTrajectories{i}(:,1), obj.vehicleTrajectories{i}(:,2), obj.vehicleTrajectories{i}(:,3), ...
                        colors{mod(i-1, length(colors))+1}, 'LineWidth', 1);
                end
            end
        end
        
        function updateView(obj)
            % Update view based on current view mode
            
            switch obj.currentView
                case 'third-person'
                    % Third-person view following the drone
                    dronePos = obj.drone.position;
                    droneDir = [cos(obj.drone.orientation(3)), sin(obj.drone.orientation(3)), 0];
                    
                    % Camera position behind and above drone
                    cameraOffset = -droneDir * 10 + [0, 0, 5];
                    obj.cameraPosition = dronePos + cameraOffset;
                    obj.cameraTarget = dronePos + droneDir * 5;
                    
                case 'drone-pov'
                    % First-person view from drone
                    dronePos = obj.drone.position;
                    
                    % Get drone rotation matrix
                    R = obj.drone.getRotationMatrix();
                    
                    % Camera direction (forward vector)
                    forwardVec = (R * [1, 0, 0]')';
                    upVec = (R * [0, 0, 1]')';
                    
                    obj.cameraPosition = dronePos;
                    obj.cameraTarget = dronePos + forwardVec * 10;
                    obj.cameraUp = upVec;
                    
                case 'top-down'
                    % Top-down view
                    bounds = obj.environment.getBounds();
                    centerX = (bounds(1,1) + bounds(1,2)) / 2;
                    centerY = (bounds(2,1) + bounds(2,2)) / 2;
                    height = max(bounds(1,2) - bounds(1,1), bounds(2,2) - bounds(2,1)) * 0.8;
                    
                    obj.cameraPosition = [centerX, centerY, height];
                    obj.cameraTarget = [centerX, centerY, 0];
                    obj.cameraUp = [0, 1, 0];
            end
            
            % Set view
            view(obj.axes, obj.cameraPosition);
            camtarget(obj.axes, obj.cameraTarget);
            camup(obj.axes, obj.cameraUp);
            
            % Set axis limits
            bounds = obj.environment.getBounds();
            xlim(obj.axes, bounds(1,:));
            ylim(obj.axes, bounds(2,:));
            zlim(obj.axes, bounds(3,:));
        end
        
        function updateHUD(obj)
            % Update HUD elements
            
            % Find closest vehicle
            minDist = Inf;
            closestVehicleIdx = 0;
            
            for i = 1:length(obj.vehicles)
                dist = norm(obj.drone.position - obj.vehicles{i}.getLandingPadPosition());
                if dist < minDist
                    minDist = dist;
                    closestVehicleIdx = i;
                end
            end
            
            % Update drone telemetry with distance to target
            obj.drone.telemetry.distanceToTarget = minDist;
            
            % Update status text
            if obj.drone.arUcoAuthenticated
                statusText = 'Landing';
            elseif closestVehicleIdx > 0
                statusText = sprintf('Tracking Vehicle %d', obj.vehicles{closestVehicleIdx}.arUcoID);
            else
                statusText = 'Searching';
            end
            
            set(obj.hudElements.statusText, 'String', ...
                sprintf('Simulation Status: %s', statusText));
            
            % Update altitude text
            set(obj.hudElements.altitudeText, 'String', ...
                sprintf('Altitude: %.2f m', obj.drone.position(3)));
            
            % Update speed text
            speed = norm(obj.drone.velocity);
            set(obj.hudElements.speedText, 'String', ...
                sprintf('Speed: %.2f m/s', speed));
            
            % Update distance text
            set(obj.hudElements.distanceText, 'String', ...
                sprintf('Distance to Nearest Vehicle: %.2f m', minDist));
            
            % Update ArUco status
            if obj.drone.arUcoAuthenticated
                arUcoStatus = sprintf('Authenticated (ID: %d)', obj.drone.authenticatedArUcoID);
            else
                arUcoStatus = 'Not Authenticated';
            end
            
            set(obj.hudElements.arUcoText, 'String', ...
                sprintf('ArUco Status: %s', arUcoStatus));
            
            % Update view mode text
            set(obj.hudElements.viewText, 'String', ...
                sprintf('View: %s', obj.currentView));
        end
        
        function toggleView(obj)
            % Toggle between different views
            
            switch obj.currentView
                case 'third-person'
                    obj.currentView = 'drone-pov';
                case 'drone-pov'
                    obj.currentView = 'top-down';
                case 'top-down'
                    obj.currentView = 'third-person';
            end
            
            disp(['Switched to ', obj.currentView, ' view']);
        end
    end
end
