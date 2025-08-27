classdef FlightDashboard < handle
    % FlightDashboard - Creates a flight instrument style display (HUD)
    
    properties
        simulation
        panel
        axes
        
        % HUD Elements
        attitudeIndicator
        horizonLine
        pitchLadder
        rollIndicator
        
        speedText
        altitudeText
        
        isVisible = true
    end
    
    methods
        function obj = FlightDashboard(simulation)
            obj.simulation = simulation;
        end
        
        function setupFlightDashboard(obj, fig)
            % Create the main panel for the flight dashboard
            obj.panel = uipanel('Parent', fig, 'Title', 'Flight Instruments', ...
                                'Position', [0.01, 0.5, 0.18, 0.48], ...
                                'BackgroundColor', [0.1 0.1 0.2], ...
                                'ForegroundColor', 'white', 'Visible', 'on');
            
            % Create the axes for the attitude indicator
            obj.axes = axes('Parent', obj.panel, 'Position', [0.1 0.25 0.8 0.7], ...
                            'Color', [0.1 0.1 0.2], 'XColor', 'none', 'YColor', 'none', ...
                            'ZColor', 'none', 'DataAspectRatio', [1 1 1]);
            
            hold(obj.axes, 'on');
            
            % Create the sky/ground background
            obj.attitudeIndicator = patch(obj.axes, [-1.5 -1.5 1.5 1.5], [-1.5 1.5 1.5 -1.5], 'b');
            set(obj.attitudeIndicator, 'FaceColor', [0.3 0.7 1.0]); % Sky color
            
            % Create the horizon line
            obj.horizonLine = plot(obj.axes, [-1.5, 1.5], [0, 0], 'w-', 'LineWidth', 2);
            
            % Create pitch ladder lines
            obj.pitchLadder = plot(obj.axes, NaN, NaN, 'w-', 'LineWidth', 1);
            
            % Create roll indicator (triangle at the top)
            obj.rollIndicator = patch(obj.axes, [0 -0.05 0.05], [1.1 1.0 1.0], 'y');
            
            % Create text displays for speed and altitude
            obj.altitudeText = text(obj.axes, 1.2, 0, 'ALT: 0 m', 'Color', 'white', 'FontSize', 10, 'HorizontalAlignment', 'right');
            obj.speedText = text(obj.axes, -1.2, 0, 'SPD: 0 m/s', 'Color', 'white', 'FontSize', 10, 'HorizontalAlignment', 'left');
            
            hold(obj.axes, 'off');
            ylim(obj.axes, [-1.2 1.2]);
            xlim(obj.axes, [-1.5 1.5]);
        end
        
        function update(obj)
            % This function updates the flight instruments
            if ~obj.isVisible || ~isgraphics(obj.panel)
                return;
            end
            
            drone = obj.simulation.drone;
            roll = drone.orientation(1);
            pitch = drone.orientation(2);
            
            % --- Update Attitude Indicator ---
            
            % Rotate the sky/ground patch based on roll
            % The patch vertices are defined in a square from -1.5 to 1.5
            sky_y = 1.5; ground_y = -1.5;
            
            % Calculate new vertices for the sky patch based on roll and pitch
            y_top_left = sky_y * cos(roll) - sky_y * sin(roll) - pitch;
            y_top_right = sky_y * cos(roll) + sky_y * sin(roll) - pitch;
            y_bot_left = ground_y * cos(roll) - sky_y * sin(roll) - pitch;
            y_bot_right = ground_y * cos(roll) + sky_y * sin(roll) - pitch;

            set(obj.attitudeIndicator, 'YData', [y_bot_left, y_top_left, y_top_right, y_bot_right]);
            
            % Update horizon line
            set(obj.horizonLine, 'YData', [-pitch -pitch]);
            set(obj.horizonLine, 'XData', [-1.5 1.5]);
            
            % Update pitch ladder (simple example)
            pitch_lines_y = [];
            pitch_lines_x = [];
            for p = -60:10:60
                if p == 0, continue; end
                y = -pitch + tand(p);
                if abs(y) < 1.2
                    pitch_lines_y = [pitch_lines_y, y, y, NaN];
                    pitch_lines_x = [pitch_lines_x, -0.2, 0.2, NaN];
                end
            end
            set(obj.pitchLadder, 'XData', pitch_lines_x, 'YData', pitch_lines_y);
            
            % Update roll indicator
            set(obj.rollIndicator, 'XData', [0, -0.05*cos(roll)-0*sin(roll), 0.05*cos(roll)-0*sin(roll)], ...
                                   'YData', [1.1, 1.0, 1.0]);

            % --- Update Text ---
            set(obj.altitudeText, 'String', sprintf('ALT: %.1f m', drone.position(3)));
            set(obj.speedText, 'String', sprintf('SPD: %.1f m/s', norm(drone.velocity)));
            
            drawnow('limitrate');
        end
        
        function toggleView(obj)
            % Toggles the visibility of the panel
            obj.isVisible = ~obj.isVisible;
            if obj.isVisible
                set(obj.panel, 'Visible', 'on');
            else
                set(obj.panel, 'Visible', 'off');
            end
        end
    end
end
