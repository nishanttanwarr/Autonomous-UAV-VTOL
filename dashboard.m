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
        altitudePlot
        speedPlot
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
        spoofingPlot % Plot for spoofing attempts
        isVisible = true
    end
    
    methods
        function obj = Dashboard(simulation)
            % Constructor
            obj.simulation = simulation;
        end
        
        function setupDashboard(obj, fig)
            % Setup dashboard panel
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
            % Setup telemetry tab
            tab = uitab(obj.tabGroup, 'Title', 'Telemetry');
            obj.telemetryPanel = uipanel('Parent', tab, 'Position', [0.01, 0.01, 0.98, 0.98]);
            axesAltitude = axes('Parent', obj.telemetryPanel, 'Position', [0.05, 0.55, 0.4, 0.4]);
            obj.altitudePlot = plot(axesAltitude, NaN, NaN, 'b-');
            title(axesAltitude, 'Altitude (m)'); xlabel('Time (s)'); ylabel('Altitude (m)');
            axesSpeed = axes('Parent', obj.telemetryPanel, 'Position', [0.55, 0.55, 0.4, 0.4]);
            obj.speedPlot = plot(axesSpeed, NaN, NaN, 'r-');
            title(axesSpeed, 'Speed (m/s)'); xlabel('Time (s)'); ylabel('Speed (m/s)');
            axesAttitude = axes('Parent', obj.telemetryPanel, 'Position', [0.3, 0.05, 0.4, 0.4]);
            obj.attitudePlot = plot(axesAttitude, NaN, NaN, 'g-');
            title(axesAttitude, 'Attitude (rad)'); xlabel('Time (s)'); ylabel('Yaw (rad)');
        end
        
        function setupVehiclesPanel(obj)
            % Setup vehicles tab
            tab = uitab(obj.tabGroup, 'Title', 'Vehicles');
            obj.vehiclesPanel = uipanel('Parent', tab, 'Position', [0.01, 0.01, 0.98, 0.98]);
            columnNames = {'Vehicle ID', 'ArUco ID', 'Position (x, y, z)', 'Speed (m/s)'};
            columnFormat = {'numeric', 'numeric', 'char', 'numeric'};
            data = cell(length(obj.simulation.vehicles), 4);
            for i = 1:length(obj.simulation.vehicles)
                data{i, 1} = i;
                data{i, 2} = obj.simulation.vehicles{i}.arUcoID;
                data{i, 3} = '[0, 0, 0]';
                data{i, 4} = 0;
            end
            obj.vehicleTable = uitable('Parent', obj.vehiclesPanel, ...
                                       'Position', [20, 20, 500, 300], ...
                                       'ColumnName', columnNames, ...
                                       'ColumnFormat', columnFormat, ...
                                       'Data', data);
        end
        
        function setupArUcoPanel(obj)
            % Setup ArUco tab
            tab = uitab(obj.tabGroup, 'Title', 'ArUco Detection');
            obj.arUcoPanel = uipanel('Parent', tab, 'Position', [0.01, 0.01, 0.98, 0.98]);
            uicontrol('Parent', obj.arUcoPanel, 'Style', 'text', ...
                      'Position', [20, 300, 200, 20], ...
                      'String', 'ArUco Detection Status:');
            obj.arUcoStatusText = uicontrol('Parent', obj.arUcoPanel, 'Style', 'text', ...
                                            'Position', [220, 300, 200, 20], ...
                                            'String', obj.simulation.arUcoDetectionStatus);
            axesArUco = axes('Parent', obj.arUcoPanel, 'Position', [0.3, 0.1, 0.4, 0.4]);
            placeholderImg = ones(100, 100, 3) * 0.5;
            obj.arUcoDetectionImage = imshow(placeholderImg, 'Parent', axesArUco);
            title(axesArUco, 'ArUco Marker');
        end
        
        function setupSecurityPanel(obj)
            % Setup security tab with adjusted layout to avoid overlap
            tab = uitab(obj.tabGroup, 'Title', 'Security (A3)');
            obj.securityPanel = uipanel('Parent', tab, 'Position', [0.01, 0.01, 0.98, 0.98]);
            uicontrol('Parent', obj.securityPanel, 'Style', 'text', ...
                      'Position', [20, 350, 150, 20], ...
                      'String', 'Authentication Status:');
            obj.authStatusLight = uicontrol('Parent', obj.securityPanel, ...
                                            'Style', 'text', ...
                                            'Position', [170, 350, 50, 20], ...
                                            'String', '🔴', ...
                                            'FontSize', 14);
            uicontrol('Parent', obj.securityPanel, 'Style', 'text', ...
                      'Position', [20, 320, 150, 20], ...
                      'String', 'Authorization Status:');
            obj.authzStatusLight = uicontrol('Parent', obj.securityPanel, ...
                                             'Style', 'text', ...
                                             'Position', [170, 320, 50, 20], ...
                                             'String', '🔴', ...
                                             'FontSize', 14);
            uicontrol('Parent', obj.securityPanel, 'Style', 'text', ...
                      'Position', [20, 290, 150, 20], ...
                      'String', 'Association Status:');
            obj.assocStatusLight = uicontrol('Parent', obj.securityPanel, ...
                                             'Style', 'text', ...
                                             'Position', [170, 290, 50, 20], ...
                                             'String', '🔴', ...
                                             'FontSize', 14);
            uicontrol('Parent', obj.securityPanel, 'Style', 'text', ...
                      'Position', [20, 260, 200, 20], ...
                      'String', 'Authentication Attempts: 0');
            uicontrol('Parent', obj.securityPanel, 'Style', 'text', ...
                      'Position', [20, 230, 200, 20], ...
                      'String', 'Association Failures: 0');
            uicontrol('Parent', obj.securityPanel, 'Style', 'text', ...
                      'Position', [20, 200, 200, 20], ...
                      'String', 'Handshake Latency: 0 s');
            uicontrol('Parent', obj.securityPanel, 'Style', 'text', ...
                      'Position', [20, 170, 200, 20], ...
                      'String', 'Spoofing Attempts: 0');
            obj.stateDiagramAxes = axes('Parent', obj.securityPanel, ...
                                        'Position', [0.5, 0.60, 0.45, 0.25], ...
                                        'Box', 'on', ...
                                        'XTick', [], 'YTick', []);
            title(obj.stateDiagramAxes, 'Association State Machine');
            axesSpoofing = axes('Parent', obj.securityPanel, ...
                                'Position', [0.5, 0.30, 0.45, 0.25]);
            obj.spoofingPlot = plot(axesSpoofing, NaN, NaN, 'r-');
            title(axesSpoofing, 'Spoofing Attempts'); xlabel('Time (s)'); ylabel('Count');
            uicontrol('Parent', obj.securityPanel, 'Style', 'text', ...
                      'Position', [20, 100, 100, 20], ...
                      'String', 'Status Log:');
            obj.statusLogText = uicontrol('Parent', obj.securityPanel, ...
                                          'Style', 'edit', ...
                                          'Position', [20, 20, 400, 80], ...
                                          'Max', 2, 'Min', 0, ...
                                          'HorizontalAlignment', 'left', ...
                                          'Enable', 'inactive');
            obj.drawStateDiagram();
        end
        
        function drawStateDiagram(obj)
            % Draw association state machine diagram
            cla(obj.stateDiagramAxes);
            axis(obj.stateDiagramAxes, [-1, 5, -1, 5]);
            hold(obj.stateDiagramAxes, 'on');
            states = {'Idle', 'Initiated', 'Confirmed', 'Failed'};
            positions = [1, 1; 3, 1; 3, 3; 1, 3];
            for i = 1:length(states)
                color = [0.8, 0.8, 0.8];
                if strcmp(obj.simulation.security.associationState, lower(states{i}))
                    color = [0, 1, 0];
                end
                rectangle('Parent', obj.stateDiagramAxes, ...
                          'Position', [positions(i,1)-0.5, positions(i,2)-0.5, 1, 1], ...
                          'Curvature', [1, 1], ...
                          'FaceColor', color, ...
                          'EdgeColor', 'k');
                text(obj.stateDiagramAxes, positions(i,1), positions(i,2), states{i}, ...
                     'HorizontalAlignment', 'center', 'FontWeight', 'bold');
            end
            obj.statePlots.arrow1 = plot(obj.stateDiagramAxes, [1.5, 2.5], [1, 1], 'k-', 'LineWidth', 1.5);
            plot(obj.stateDiagramAxes, 2.5, 1, 'k>', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
            obj.statePlots.arrow2 = plot(obj.stateDiagramAxes, [3, 3], [1.5, 2.5], 'k-', 'LineWidth', 1.5);
            plot(obj.stateDiagramAxes, 3, 2.5, 'k^', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
            obj.statePlots.arrow3 = plot(obj.stateDiagramAxes, [2.5, 1.5], [1.5, 2.5], 'k-', 'LineWidth', 1.5);
            plot(obj.stateDiagramAxes, 1.5, 2.5, 'k<', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
            obj.statePlots.arrow4 = plot(obj.stateDiagramAxes, [1, 1], [2.5, 1.5], 'k-', 'LineWidth', 1.5);
            plot(obj.stateDiagramAxes, 1, 1.5, 'kv', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
            hold(obj.stateDiagramAxes, 'off');
        end
        
        function update(obj)
            % Update dashboard based on current tab
            if ~obj.isVisible
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
        
        function updateTelemetryPlots(obj)
            % Update telemetry plots
            time = obj.simulation.currentTime;
            altitude = obj.simulation.drone.position(3);
            speed = norm(obj.simulation.drone.velocity);
            attitude = obj.simulation.drone.orientation(3);
            xData = get(obj.altitudePlot, 'XData');
            yData = get(obj.altitudePlot, 'YData');
            xData = [xData, time];
            yData = [yData, altitude];
            if length(xData) > 100
                xData = xData(end-99:end);
                yData = yData(end-99:end);
            end
            set(obj.altitudePlot, 'XData', xData, 'YData', yData);
            xData = get(obj.speedPlot, 'XData');
            yData = get(obj.speedPlot, 'YData');
            xData = [xData, time];
            yData = [yData, speed];
            if length(xData) > 100
                xData = xData(end-99:end);
                yData = yData(end-99:end);
            end
            set(obj.speedPlot, 'XData', xData, 'YData', yData);
            xData = get(obj.attitudePlot, 'XData');
            yData = get(obj.attitudePlot, 'YData');
            xData = [xData, time];
            yData = [yData, attitude];
            if length(xData) > 100
                xData = xData(end-99:end);
                yData = yData(end-99:end);
            end
            set(obj.attitudePlot, 'XData', xData, 'YData', yData);
        end
        
        function updateVehiclesPanel(obj)
            % Update vehicles panel
            data = get(obj.vehicleTable, 'Data');
            for i = 1:length(obj.simulation.vehicles)
                vehicle = obj.simulation.vehicles{i};
                pos = vehicle.position;
                data{i, 3} = sprintf('[%.1f, %.1f, %.1f]', pos(1), pos(2), pos(3));
                data{i, 4} = norm(vehicle.velocity);
            end
            set(obj.vehicleTable, 'Data', data);
        end
        
        function updateArUcoPanel(obj)
            % Update ArUco panel
            set(obj.arUcoStatusText, 'String', obj.simulation.arUcoDetectionStatus);
            detectedID = obj.simulation.getDetectedArUcoID();
            if obj.simulation.drone.arUcoAuthenticated
                arUcoID = obj.simulation.drone.authenticatedArUcoID;
                img = obj.createArUcoImage(arUcoID);
                set(obj.arUcoDetectionImage, 'CData', img);
            elseif detectedID > 0
                img = obj.createArUcoImage(detectedID);
                set(obj.arUcoDetectionImage, 'CData', img);
            else
                placeholderImg = ones(100, 100, 3) * 0.5;
                set(obj.arUcoDetectionImage, 'CData', placeholderImg);
            end
        end
        
        function updateSecurityPanel(obj)
            % Update security panel
            if obj.simulation.drone.arUcoAuthenticated
                set(obj.authStatusLight, 'String', '🟢', 'ForegroundColor', [0, 1, 0]);
                obj.addStatusLog('Authentication successful');
            else
                set(obj.authStatusLight, 'String', '🔴', 'ForegroundColor', [1, 0, 0]);
            end
            if obj.simulation.security.isAuthorized
                set(obj.authzStatusLight, 'String', '🟢', 'ForegroundColor', [0, 1, 0]);
                obj.addStatusLog('Authorization successful');
            else
                set(obj.authzStatusLight, 'String', '🔴', 'ForegroundColor', [1, 0, 0]);
            end
            switch obj.simulation.security.associationState
                case 'confirmed'
                    set(obj.assocStatusLight, 'String', '🟢', 'ForegroundColor', [0, 1, 0]);
                    obj.addStatusLog('Association confirmed');
                case 'failed'
                    set(obj.assocStatusLight, 'String', '🔴', 'ForegroundColor', [1, 0, 0]);
                    obj.addStatusLog('Association failed');
                    if obj.simulation.security.spoofingDetected
                        obj.addStatusLog('Spoofing attempt detected');
                    end
                otherwise
                    set(obj.assocStatusLight, 'String', '🟡', 'ForegroundColor', [1, 1, 0]);
                    if strcmp(obj.simulation.security.associationState, 'initiated')
                        obj.addStatusLog('Association initiated');
                    end
            end
            metrics = obj.simulation.security.getSecurityMetrics();
            uicontrol('Parent', obj.securityPanel, 'Style', 'text', ...
                      'Position', [20, 260, 200, 20], ...
                      'String', sprintf('Authentication Attempts: %d', metrics.authAttempts));
            uicontrol('Parent', obj.securityPanel, 'Style', 'text', ...
                      'Position', [20, 230, 200, 20], ...
                      'String', sprintf('Association Failures: %d', metrics.associationFailures));
            uicontrol('Parent', obj.securityPanel, 'Style', 'text', ...
                      'Position', [20, 200, 200, 20], ...
                      'String', sprintf('Handshake Latency: %.3f s', metrics.handshakeLatency));
            uicontrol('Parent', obj.securityPanel, 'Style', 'text', ...
                      'Position', [20, 170, 200, 20], ...
                      'String', sprintf('Spoofing Attempts: %d', metrics.spoofingAttempts));
            % Update spoofing attempts plot
            xData = get(obj.spoofingPlot, 'XData');
            yData = get(obj.spoofingPlot, 'YData');
            xData = [xData, obj.simulation.currentTime];
            yData = [yData, metrics.spoofingAttempts];
            if length(xData) > 100
                xData = xData(end-99:end);
                yData = yData(end-99:end);
            end
            set(obj.spoofingPlot, 'XData', xData, 'YData', yData);
            obj.drawStateDiagram();
        end
        
        function addStatusLog(obj, message)
            % Add message to status log with timestamp
            currentLog = get(obj.statusLogText, 'String');
            timestamp = datestr(now, 'HH:MM:SS');
            newEntry = sprintf('%s: %s', timestamp, message);
            if iscell(currentLog)
                currentLog = [currentLog; newEntry];
            else
                currentLog = {currentLog; newEntry};
            end
            if length(currentLog) > 10
                currentLog = currentLog(end-9:end);
            end
            set(obj.statusLogText, 'String', currentLog);
        end
        
        function img = createArUcoImage(obj, arUcoID)
            % Create a black-and-white ArUco-like marker image
            img = ones(100, 100, 3);
            gridSize = 5;
            cellSize = round(100 / (gridSize + 2));
            border = cellSize;
            totalSize = (gridSize + 2) * cellSize;
            offset = floor((100 - totalSize) / 2);
            border = border + offset;
            for ch = 1:3
                img(:,:,ch) = 1;
            end
            for ch = 1:3
                img(1:border, :, ch) = 0;
                img(end-border+1:end, :, ch) = 0;
                img(:, 1:border, ch) = 0;
                img(:, end-border+1:end, ch) = 0;
            end
            if arUcoID == 1
                pattern = [1 0 1 0 1; ...
                           0 1 0 1 0; ...
                           1 0 1 0 1; ...
                           0 1 0 1 0; ...
                           1 0 1 0 1];
            elseif arUcoID == 2
                pattern = [0 1 1 1 0; ...
                           1 0 0 0 1; ...
                           1 0 0 0 1; ...
                           1 0 0 0 1; ...
                           0 1 1 1 0];
            elseif arUcoID == 3
                pattern = [1 1 0 1 1; ...
                           1 0 1 0 1; ...
                           0 1 0 1 0; ...
                           1 0 1 0 1; ...
                           1 1 0 1 1];
            else
                pattern = ones(gridSize, gridSize);
            end
            for i = 1:gridSize
                for j = 1:gridSize
                    if pattern(i, j) == 0
                        rowStart = border + (i-1) * cellSize + 1;
                        rowEnd = border + i * cellSize;
                        colStart = border + (j-1) * cellSize + 1;
                        colEnd = border + j * cellSize;
                        for ch = 1:3
                            img(rowStart:rowEnd, colStart:colEnd, ch) = 0;
                        end
                    end
                end
            end
        end
        
        function toggleView(obj)
            % Toggle dashboard visibility
            obj.isVisible = ~obj.isVisible;
            if obj.isVisible
                set(obj.dashboardPanel, 'Visible', 'on');
            else
                set(obj.dashboardPanel, 'Visible', 'off');
            end
        end
    end
end
