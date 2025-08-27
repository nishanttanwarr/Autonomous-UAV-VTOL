classdef Dashboard < handle
    % Dashboard - Professional dashboard for monitoring simulation
    
    properties
        simulation
        dashboardPanel
        tabGroup
        telemetryPanel
        vehiclesPanel
        arUcoPanel
        securityPanel
        currentTab = 'telemetry'
        
        % Telemetry plots
        trueAltitudePlot
        ekfAltitudePlot
        trueYawPlot
        ekfYawPlot
        speedPlot
        
        estimationErrorText    % Text handle for position error
        
        attitudePlot
        arUcoDetectionImage
        arUcoStatusText
        authStatusLight
        authzStatusLight
        assocStatusLight
        statusLogText
        stateDiagramAxes
        statePlots
        vehicleTable
        vehicleSpeedSliders
        spoofingPlot % Plot for spoofing attempts
        isVisible = true
    end
    methods
        function obj = Dashboard(simulation)
            obj.simulation = simulation;
        end
        
        function setupDashboard(obj, fig)
            obj.dashboardPanel = uipanel('Parent', fig, ...
                                         'Title', 'Simulation Dashboard', ...
                                         'Position', [0.6, 0.01, 0.39, 0.98], ...
                                         'BackgroundColor', [0.95, 0.95, 0.95], ...
                                         'Visible', 'on');
            obj.tabGroup = uitabgroup('Parent', obj.dashboardPanel, ...
                                      'Position', [0.01, 0.01, 0.98, 0.98]);
            obj.setupTelemetryPanel();
            obj.setupVehiclesPanel();
            obj.setupArUcoPanel();
            obj.setupSecurityPanel();
        end
        
        function setupTelemetryPanel(obj)
            tab = uitab(obj.tabGroup, 'Title', 'Telemetry');
            obj.telemetryPanel = uipanel('Parent', tab, 'Position', [0.01, 0.01, 0.98, 0.98]);
            % True vs EKF Altitude
            axesAlt = axes('Parent', obj.telemetryPanel, 'Position', [0.05, 0.55, 0.4, 0.4]);
            hold(axesAlt, 'on');
            obj.trueAltitudePlot = plot(axesAlt, NaN, NaN, 'b-', 'DisplayName', 'True Altitude');
            obj.ekfAltitudePlot  = plot(axesAlt, NaN, NaN, 'r--', 'DisplayName', 'EKF Altitude');
            legend(axesAlt); hold(axesAlt, 'off');
            title(axesAlt, 'Altitude (m)'); xlabel(axesAlt, 'Time (s)'); ylabel(axesAlt, 'Altitude (m)');
            % True vs EKF Yaw
            axesYaw = axes('Parent', obj.telemetryPanel, 'Position', [0.55, 0.55, 0.4, 0.4]);
            hold(axesYaw, 'on');
            obj.trueYawPlot = plot(axesYaw, NaN, NaN, 'b-', 'DisplayName', 'True Yaw');
            obj.ekfYawPlot  = plot(axesYaw, NaN, NaN, 'r--', 'DisplayName', 'EKF Yaw');
            legend(axesYaw); hold(axesYaw, 'off');
            title(axesYaw, 'Yaw (deg)'); xlabel(axesYaw, 'Time (s)'); ylabel(axesYaw, 'Yaw (deg)');
            % Speed
            axesSpeed = axes('Parent', obj.telemetryPanel, 'Position', [0.30, 0.05, 0.4, 0.4]);
            obj.speedPlot = plot(axesSpeed, NaN, NaN, 'g-');
            title(axesSpeed, 'Speed (m/s)'); xlabel(axesSpeed, 'Time (s)'); ylabel(axesSpeed, 'Speed (m/s)');
            % Estimation error text
            obj.estimationErrorText = uicontrol('Parent', obj.telemetryPanel, ...
                'Style', 'text', 'Position', [150, 45, 200, 22], ...
                'String', 'Estimation Error: N/A', 'FontSize', 10, 'HorizontalAlignment', 'left');
        end
        
        function setupVehiclesPanel(obj)
            tab = uitab(obj.tabGroup, 'Title', 'Vehicles');
            obj.vehiclesPanel = uipanel('Parent', tab, 'Position', [0.01, 0.01, 0.98, 0.98]);
            
            % Table
            columnNames = {'Vehicle ID', 'ArUco ID', 'Position (x,y,z)', 'Target v (m/s)', 'Actual v (m/s)'};
            columnFormat = {'numeric', 'numeric', 'char', 'numeric', 'numeric'};
            data = cell(length(obj.simulation.vehicles), 5);
            for i = 1:length(obj.simulation.vehicles)
                v = obj.simulation.vehicles{i};
                data{i, 1} = i;
                data{i, 2} = v.arUcoID;
                data{i, 3} = mat2str(v.position, 3);
                data{i, 4} = v.targetSpeed;
                data{i, 5} = v.curSpeed;
            end
            obj.vehicleTable = uitable('Parent', obj.vehiclesPanel, ...
                                       'Position', [20, 150, 500, 200], ...
                                       'ColumnName', columnNames, ...
                                       'ColumnFormat', columnFormat, ...
                                       'Data', data);
            
            % Sliders for manual control
            sliderHeight = 0.04;
            sliderWidth  = 0.35;
            sliderX      = 0.65;   
            startY       = 0.85;
            obj.vehicleSpeedSliders = gobjects(length(obj.simulation.vehicles),1);
            
            for vi = 1:numel(obj.simulation.vehicles)
                uicontrol('Parent', obj.vehiclesPanel, 'Style', 'text', 'Units', 'normalized', ...
                    'Position', [sliderX, startY - (vi-1)*(sliderHeight+0.01) + 0.02, sliderWidth, 0.02], ...
                    'String', sprintf('Vehicle %d Target Speed', vi), 'BackgroundColor', [0.9 0.9 0.9], 'FontSize', 9);
                
                obj.vehicleSpeedSliders(vi) = uicontrol('Parent', obj.vehiclesPanel, 'Style', 'slider', ...
                    'Min', 0, 'Max', 20, 'Value', obj.simulation.vehicles{vi}.targetSpeed, 'Units', 'normalized', ...
                    'Position', [sliderX, startY - (vi-1)*(sliderHeight+0.01), sliderWidth, sliderHeight], ...
                    'Callback', @(h,~) obj.setVehicleTargetSpeed(vi, get(h,'Value')));
            end
        end
        
        function setupArUcoPanel(obj)
            tab = uitab(obj.tabGroup, 'Title', 'ArUco Detection');
            obj.arUcoPanel = uipanel('Parent', tab, 'Position', [0.01, 0.01, 0.98, 0.98]);
            uicontrol('Parent', obj.arUcoPanel, 'Style', 'text', 'Position', [20, 300, 200, 20], 'String', 'ArUco Detection Status:');
            obj.arUcoStatusText = uicontrol('Parent', obj.arUcoPanel, 'Style', 'text', 'Position', [220, 300, 200, 20], 'String', obj.simulation.arUcoDetectionStatus);
            axesArUco = axes('Parent', obj.arUcoPanel, 'Position', [0.3, 0.1, 0.4, 0.4]);
            placeholderImg = ones(100, 100, 3) * 0.5;
            obj.arUcoDetectionImage = imshow(placeholderImg, 'Parent', axesArUco);
            title(axesArUco, 'ArUco Marker');
        end
        
        function setupSecurityPanel(obj)
            % This is a placeholder. You would add your security UI elements here.
            tab = uitab(obj.tabGroup, 'Title', 'Security (A3)');
            obj.securityPanel = uipanel('Parent', tab, 'Position', [0.01, 0.01, 0.98, 0.98]);
            obj.statusLogText = uicontrol('Parent', obj.securityPanel, 'Style', 'text', 'Position', [20, 20, 400, 300], 'String', 'Security Log:', 'HorizontalAlignment', 'left');
        end
        
        function setVehicleTargetSpeed(obj, vehicleIdx, newSpeed)
            v = obj.simulation.vehicles{vehicleIdx};
            v.targetSpeed = newSpeed;
            v.autoProfile = false;  % disable auto variation
        end
        
        function update(obj)
            if ~obj.isVisible || ~isvalid(obj.dashboardPanel)
                return;
            end
            obj.currentTab = obj.tabGroup.SelectedTab.Title;
            switch obj.currentTab
                case 'Telemetry'
                    obj.updateTelemetryPlots();
                case 'Vehicles'
                    obj.updateVehiclesPanel();
                case 'ArUco Detection'
                    obj.updateArUcoPanel();
                case 'Security (A3)'
                    obj.updateSecurityPanel();
            end
        end
        
        function updateVehiclesPanel(obj)
            % Refresh the vehicles table
            data = cell(length(obj.simulation.vehicles), 5);
            for i = 1:length(obj.simulation.vehicles)
                v = obj.simulation.vehicles{i};
                data{i, 1} = i;
                data{i, 2} = v.arUcoID;
                data{i, 3} = mat2str(v.position, 3);
                data{i, 4} = round(v.targetSpeed,2);
                data{i, 5} = round(v.curSpeed,2);
                
                if isgraphics(obj.vehicleSpeedSliders(i))
                    set(obj.vehicleSpeedSliders(i), 'Value', v.targetSpeed);
                end
            end
            set(obj.vehicleTable, 'Data', data);
        end

        %%% ADDED TO FIX ERROR %%%
        function updateTelemetryPlots(obj)
            % This function updates the data in the telemetry plots
            
            % Check if plot handles are valid graphics objects
            if ~isgraphics(obj.trueAltitudePlot) || ~isgraphics(obj.ekfAltitudePlot)
                return; % Exit if plots aren't set up correctly
            end

            % Get current simulation time and drone telemetry
            simTime = obj.simulation.currentTime;
            drone = obj.simulation.drone;
            
            % Update Altitude Plot by appending new data
            set(obj.trueAltitudePlot, 'XData', [get(obj.trueAltitudePlot, 'XData'), simTime], ...
                                      'YData', [get(obj.trueAltitudePlot, 'YData'), drone.position(3)]);
            set(obj.ekfAltitudePlot, 'XData', [get(obj.ekfAltitudePlot, 'XData'), simTime], ...
                                     'YData', [get(obj.ekfAltitudePlot, 'YData'), drone.telemetry.ekf_altitude]);

            % Update Yaw Plot
            set(obj.trueYawPlot, 'XData', [get(obj.trueYawPlot, 'XData'), simTime], ...
                                  'YData', [get(obj.trueYawPlot, 'YData'), drone.orientation(3) * 180/pi]);
            set(obj.ekfYawPlot, 'XData', [get(obj.ekfYawPlot, 'XData'), simTime], ...
                                 'YData', [get(obj.ekfYawPlot, 'YData'), drone.telemetry.ekf_yaw]);

            % Update Speed Plot
            set(obj.speedPlot, 'XData', [get(obj.speedPlot, 'XData'), simTime], ...
                               'YData', [get(obj.speedPlot, 'YData'), norm(drone.velocity)]);

            % Update Estimation Error Text
            pos_error = norm(drone.position - drone.ekf_estimated_state(1:3)');
            set(obj.estimationErrorText, 'String', sprintf('Estimation Error: %.2f m', pos_error));
            
            drawnow('limitrate');
        end

        function updateArUcoPanel(obj)
            % This is a placeholder. You would update the ArUco status text and image here.
            set(obj.arUcoStatusText, 'String', obj.simulation.arUcoDetectionStatus);
        end

        function updateSecurityPanel(obj)
            % This is a placeholder. You would update security status lights and logs here.
            % For example:
            % set(obj.statusLogText, 'String', obj.simulation.security.getLogs());
        end
        
        function addStatusLog(obj, message)
            % Helper to add messages to the security log
            currentLog = get(obj.statusLogText, 'String');
            if ~iscell(currentLog)
                currentLog = {currentLog};
            end
            newLog = [currentLog; {sprintf('T+%.2f: %s', obj.simulation.currentTime, message)}];
            set(obj.statusLogText, 'String', newLog);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
end
