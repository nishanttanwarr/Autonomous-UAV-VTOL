classdef FlightDashboard < handle
    % FlightDashboard - Professional flight dashboard for monitoring drone orientation and flight details
    
    properties
        simulation
        flightPanel
        horizonAxes
        compassAxes
        rollText
        pitchText
        yawText
        altitudeText
        speedText
        batteryText
        distanceText
        flightModeText
        horizonLine
        compassNeedle
        isVisible = true
    end
    
    methods
        function obj = FlightDashboard(simulation)
            % Constructor
            obj.simulation = simulation;
        end
        
        function setupFlightDashboard(obj, fig)
            % Setup flight dashboard panel
            % Position the flight dashboard on the bottom left (below visualizer)
            obj.flightPanel = uipanel('Parent', fig, ...
                                      'Title', 'Flight Dashboard', ...
                                      'Position', [0.01, 0.01, 0.58, 0.29], ...
                                      'BackgroundColor', [0.95, 0.95, 0.95], ...
                                      'Visible', 'on');
            
            % Setup horizon (roll and pitch indicator)
            obj.horizonAxes = axes('Parent', obj.flightPanel, ...
                                   'Position', [0.05, 0.5, 0.25, 0.4], ...
                                   'Box', 'on', 'XTick', [], 'YTick', []);
            axis(obj.horizonAxes, [-1 1 -1 1]);
            hold(obj.horizonAxes, 'on');
            title(obj.horizonAxes, 'Attitude Indicator');
            % Draw horizon line (will be updated dynamically)
            obj.horizonLine = plot(obj.horizonAxes, [-1 1], [0 0], 'b-', 'LineWidth', 2);
            
            % Setup compass (yaw indicator)
            obj.compassAxes = axes('Parent', obj.flightPanel, ...
                                   'Position', [0.35, 0.5, 0.25, 0.4], ...
                                   'Box', 'on', 'XTick', [], 'YTick', []);
            axis(obj.compassAxes, [-1 1 -1 1]);
            hold(obj.compassAxes, 'on');
            title(obj.compassAxes, 'Heading');
            % Draw compass circle
            theta = linspace(0, 2*pi, 100);
            plot(obj.compassAxes, cos(theta), sin(theta), 'k-');
            % Draw cardinal directions
            text(obj.compassAxes, 0, 1.1, 'N', 'HorizontalAlignment', 'center');
            text(obj.compassAxes, 0, -1.1, 'S', 'HorizontalAlignment', 'center');
            text(obj.compassAxes, 1.1, 0, 'E', 'HorizontalAlignment', 'center');
            text(obj.compassAxes, -1.1, 0, 'W', 'HorizontalAlignment', 'center');
            % Draw compass needle (will be updated dynamically)
            obj.compassNeedle = plot(obj.compassAxes, [0 0], [0 0.8], 'r-', 'LineWidth', 2);
            
            % Setup text displays
            % Orientation values
            uicontrol('Parent', obj.flightPanel, 'Style', 'text', ...
                      'Position', [20, 50, 80, 20], ...
                      'String', 'Roll:');
            obj.rollText = uicontrol('Parent', obj.flightPanel, 'Style', 'text', ...
                                     'Position', [100, 50, 80, 20], ...
                                     'String', '0 deg');
            uicontrol('Parent', obj.flightPanel, 'Style', 'text', ...
                      'Position', [20, 30, 80, 20], ...
                      'String', 'Pitch:');
            obj.pitchText = uicontrol('Parent', obj.flightPanel, 'Style', 'text', ...
                                      'Position', [100, 30, 80, 20], ...
                                      'String', '0 deg');
            uicontrol('Parent', obj.flightPanel, 'Style', 'text', ...
                      'Position', [20, 10, 80, 20], ...
                      'String', 'Yaw:');
            obj.yawText = uicontrol('Parent', obj.flightPanel, 'Style', 'text', ...
                                    'Position', [100, 10, 80, 20], ...
                                    'String', '0 deg');
            
            % Flight details
            uicontrol('Parent', obj.flightPanel, 'Style', 'text', ...
                      'Position', [200, 50, 100, 20], ...
                      'String', 'Altitude:');
            obj.altitudeText = uicontrol('Parent', obj.flightPanel, 'Style', 'text', ...
                                         'Position', [300, 50, 80, 20], ...
                                         'String', '0 m');
            uicontrol('Parent', obj.flightPanel, 'Style', 'text', ...
                      'Position', [200, 30, 100, 20], ...
                      'String', 'Speed:');
            obj.speedText = uicontrol('Parent', obj.flightPanel, 'Style', 'text', ...
                                      'Position', [300, 30, 80, 20], ...
                                      'String', '0 m/s');
            uicontrol('Parent', obj.flightPanel, 'Style', 'text', ...
                      'Position', [200, 10, 100, 20], ...
                      'String', 'Battery:');
            obj.batteryText = uicontrol('Parent', obj.flightPanel, 'Style', 'text', ...
                                        'Position', [300, 10, 80, 20], ...
                                        'String', '100 %');
            uicontrol('Parent', obj.flightPanel, 'Style', 'text', ...
                      'Position', [400, 50, 100, 20], ...
                      'String', 'Distance to Target:');
            obj.distanceText = uicontrol('Parent', obj.flightPanel, 'Style', 'text', ...
                                         'Position', [500, 50, 80, 20], ...
                                         'String', 'Inf m');
            uicontrol('Parent', obj.flightPanel, 'Style', 'text', ...
                      'Position', [400, 30, 100, 20], ...
                      'String', 'Flight Mode:');
            obj.flightModeText = uicontrol('Parent', obj.flightPanel, 'Style', 'text', ...
                                           'Position', [500, 30, 80, 20], ...
                                           'String', 'Autonomous');
        end
        
        function update(obj)
            % Update flight dashboard
            if ~obj.isVisible
                return;
            end
            
            % Update orientation
            roll = obj.simulation.drone.orientation(1) * 180/pi;
            pitch = obj.simulation.drone.orientation(2) * 180/pi;
            yaw = obj.simulation.drone.orientation(3) * 180/pi;
            
            % Update horizon indicator (roll and pitch)
            cla(obj.horizonAxes);
            hold(obj.horizonAxes, 'on');
            axis(obj.horizonAxes, [-1 1 -1 1]);
            title(obj.horizonAxes, 'Attitude Indicator');
            % Sky (blue) and ground (brown)
            fill(obj.horizonAxes, [-1 1 1 -1], [0 0 1 1], [0.2, 0.6, 1], 'EdgeColor', 'none');
            fill(obj.horizonAxes, [-1 1 1 -1], [0 0 -1 -1], [0.6, 0.4, 0.2], 'EdgeColor', 'none');
            % Horizon line rotated by roll and shifted by pitch
            rollRad = roll * pi/180;
            pitchShift = pitch / 45; % Scale pitch to fit within axes
            x = [-1 1];
            y = [0 0] + pitchShift;
            % Rotate by roll
            x_rot = x * cos(rollRad) - y * sin(rollRad);
            y_rot = x * sin(rollRad) + y * cos(rollRad);
            plot(obj.horizonAxes, x_rot, y_rot, 'b-', 'LineWidth', 2);
            hold(obj.horizonAxes, 'off');
            
            % Update compass (yaw)
            cla(obj.compassAxes);
            hold(obj.compassAxes, 'on');
            axis(obj.compassAxes, [-1 1 -1 1]);
            title(obj.compassAxes, 'Heading');
            % Redraw compass circle
            theta = linspace(0, 2*pi, 100);
            plot(obj.compassAxes, cos(theta), sin(theta), 'k-');
            text(obj.compassAxes, 0, 1.1, 'N', 'HorizontalAlignment', 'center');
            text(obj.compassAxes, 0, -1.1, 'S', 'HorizontalAlignment', 'center');
            text(obj.compassAxes, 1.1, 0, 'E', 'HorizontalAlignment', 'center');
            text(obj.compassAxes, -1.1, 0, 'W', 'HorizontalAlignment', 'center');
            % Update needle
            yawRad = -yaw * pi/180; % Negative because MATLAB's plot rotates clockwise
            needleX = [0, 0.8 * cos(yawRad)];
            needleY = [0, 0.8 * sin(yawRad)];
            plot(obj.compassAxes, needleX, needleY, 'r-', 'LineWidth', 2);
            hold(obj.compassAxes, 'off');
            
            % Update text displays
            set(obj.rollText, 'String', sprintf('%.1f deg', roll));
            set(obj.pitchText, 'String', sprintf('%.1f deg', pitch));
            set(obj.yawText, 'String', sprintf('%.1f deg', yaw));
            set(obj.altitudeText, 'String', sprintf('%.1f m', obj.simulation.drone.position(3)));
            set(obj.speedText, 'String', sprintf('%.1f m/s', norm(obj.simulation.drone.velocity)));
            set(obj.batteryText, 'String', sprintf('%.1f %%', obj.simulation.drone.telemetry.batteryLevel));
            
            % Calculate distance to target
            if obj.simulation.targetVehicleIndex > 0
                targetPos = obj.simulation.vehicles{obj.simulation.targetVehicleIndex}.getLandingPadPosition();
                distance = norm(obj.simulation.drone.position - targetPos);
                set(obj.distanceText, 'String', sprintf('%.1f m', distance));
            else
                set(obj.distanceText, 'String', 'Inf m');
            end
            
            % Update flight mode
            if obj.simulation.isAutonomous
                set(obj.flightModeText, 'String', 'Autonomous');
            else
                set(obj.flightModeText, 'String', 'Manual');
            end
        end
        
        function toggleView(obj)
            % Toggle flight dashboard visibility
            obj.isVisible = ~obj.isVisible;
            if obj.isVisible
                set(obj.flightPanel, 'Visible', 'on');
            else
                set(obj.flightPanel, 'Visible', 'off');
            end
        end
    end
end
