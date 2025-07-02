classdef Environment < handle
    % Environment - Class representing the 3D environment
    
    properties
        % Environment bounds [min_x, max_x; min_y, max_y; min_z, max_z]
        bounds = [-100, 100; -100, 100; 0, 50]
        
        % Terrain properties
        terrain
        
        % Obstacles
        obstacles = []  % Format: [x, y, z, radius]
        
        % Weather conditions
        windSpeed = [0, 0, 0]
        turbulenceIntensity = 0.1
    end
    
    methods
        function obj = Environment()
            % Constructor
            
            % Initialize terrain
            obj.initializeTerrain();
            
            % Add random obstacles
            obj.addRandomObstacles(10);
        end
        
        function initializeTerrain(obj)
            % Initialize terrain
            
            % Create grid
            [X, Y] = meshgrid(linspace(obj.bounds(1,1), obj.bounds(1,2), 50), ...
                             linspace(obj.bounds(2,1), obj.bounds(2,2), 50));
            
            % Create height map (flat with some random variations)
            Z = zeros(size(X)) + 0.5 * perlin2d(X/50, Y/50);
            
            % Store terrain
            obj.terrain.X = X;
            obj.terrain.Y = Y;
            obj.terrain.Z = Z;
        end
        
        function addRandomObstacles(obj, numObstacles)
            % Add random obstacles to the environment
            
            obj.obstacles = zeros(numObstacles, 4);
            
            for i = 1:numObstacles
                % Random position within bounds
                x = obj.bounds(1,1) + (obj.bounds(1,2) - obj.bounds(1,1)) * rand();
                y = obj.bounds(2,1) + (obj.bounds(2,2) - obj.bounds(2,1)) * rand();
                z = 5 + 15 * rand();  % Obstacles at various heights
                
                % Random radius
                radius = 2 + 3 * rand();
                
                % Add obstacle
                obj.obstacles(i, :) = [x, y, z, radius];
            end
        end
        
        function bounds = getBounds(obj)
            % Get environment bounds
            bounds = obj.bounds;
        end
        
        function obstacles = getObstacles(obj)
            % Get obstacles
            obstacles = obj.obstacles;
        end
        
        function [windVelocity, turbulence] = getWindAt(obj, position)
            % Get wind velocity and turbulence at a given position
            
            % Base wind
            windVelocity = obj.windSpeed;
            
            % Add position-dependent variations
            windVelocity = windVelocity + obj.turbulenceIntensity * [
                sin(0.1 * position(1) + 0.3 * position(2)),
                cos(0.2 * position(1) - 0.1 * position(3)),
                sin(0.1 * position(2) + 0.2 * position(3))
            ];
            
            % Calculate turbulence intensity
            turbulence = obj.turbulenceIntensity * (1 + 0.5 * sin(0.05 * sum(position)));
        end
        
        function drawEnvironment(obj, ax)
            % Draw environment in the given axes
            
            % Draw terrain
            surf(ax, obj.terrain.X, obj.terrain.Y, obj.terrain.Z, ...
                'FaceColor', [0.5, 0.8, 0.2], 'EdgeColor', 'none', 'FaceAlpha', 0.8);
            
            % Draw obstacles
            for i = 1:size(obj.obstacles, 1)
                % Extract obstacle properties
                center = obj.obstacles(i, 1:3);
                radius = obj.obstacles(i, 4);
                
                % Create sphere
                [X, Y, Z] = sphere(20);
                X = X * radius + center(1);
                Y = Y * radius + center(2);
                Z = Z * radius + center(3);
                
                % Draw obstacle
                surf(ax, X, Y, Z, 'FaceColor', [0.7, 0.7, 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
            end
            
            % Draw environment bounds
            x_min = obj.bounds(1, 1);
            x_max = obj.bounds(1, 2);
            y_min = obj.bounds(2, 1);
            y_max = obj.bounds(2, 2);
            z_min = obj.bounds(3, 1);
            z_max = obj.bounds(3, 2);
            
            % Draw bounding box (wireframe)
            plot3(ax, [x_min, x_max, x_max, x_min, x_min], ...
                     [y_min, y_min, y_max, y_max, y_min], ...
                     [z_min, z_min, z_min, z_min, z_min], 'k--');
            plot3(ax, [x_min, x_max, x_max, x_min, x_min], ...
                     [y_min, y_min, y_max, y_max, y_min], ...
                     [z_max, z_max, z_max, z_max, z_max], 'k--');
            for i = 1:4
                x = [x_min, x_max, x_max, x_min];
                y = [y_min, y_min, y_max, y_max];
                plot3(ax, [x(i), x(i)], [y(i), y(i)], [z_min, z_max], 'k--');
            end
        end
    end
end

function z = perlin2d(x, y)
    % Simple 2D Perlin noise implementation
    
    % Initialize output
    z = zeros(size(x));
    
    % Multiple octaves
    amplitude = 1.0;
    frequency = 1.0;
    persistence = 0.5;
    
    for octave = 1:4
        z = z + amplitude * simplex2d(x * frequency, y * frequency);
        amplitude = amplitude * persistence;
        frequency = frequency * 2;
    end
end

function z = simplex2d(x, y)
    % Simplified 2D simplex noise
    
    % Constants
    F2 = 0.5 * (sqrt(3) - 1);
    G2 = (3 - sqrt(3)) / 6;
    
    % Skew input space
    s = (x + y) * F2;
    i = floor(x + s);
    j = floor(y + s);
    
    % Unskew
    t = (i + j) * G2;
    X0 = i - t;
    Y0 = j - t;
    x0 = x - X0;
    y0 = y - Y0;
    
    % Determine simplex
    if x0 > y0
        i1 = 1; j1 = 0;
    else
        i1 = 0; j1 = 1;
    end
    
    % Offsets for other corners
    x1 = x0 - i1 + G2;
    y1 = y0 - j1 + G2;
    x2 = x0 - 1 + 2 * G2;
    y2 = y0 - 1 + 2 * G2;
    
    % Hashed gradient indices
    ii = mod(i, 256);
    jj = mod(j, 256);
    
    % Calculate contribution from each corner
    t0 = 0.5 - x0*x0 - y0*y0;
    if t0 < 0
        n0 = 0;
    else
        t0 = t0 * t0;
        n0 = t0 * t0 * grad2d(hash(ii, jj), x0, y0);
    end
    
    t1 = 0.5 - x1*x1 - y1*y1;
    if t1 < 0
        n1 = 0;
    else
        t1 = t1 * t1;
        n1 = t1 * t1 * grad2d(hash(ii + i1, jj + j1), x1, y1);
    end
    
    t2 = 0.5 - x2*x2 - y2*y2;
    if t2 < 0
        n2 = 0;
    else
        t2 = t2 * t2;
        n2 = t2 * t2 * grad2d(hash(ii + 1, jj + 1), x2, y2);
    end
    
    % Scale to [-1,1]
    z = 70 * (n0 + n1 + n2);
end

function h = hash(i, j)
    % Simple hash function
    h = mod(i * 73856093 + j * 19349663, 256);
end

function g = grad2d(hash, x, y)
    % 2D gradient function
    h = mod(hash, 8);
    
    if h < 4
        u = x;
    else
        u = y;
    end
    
    if h < 2
        v = y;
    else
        v = x;
    end
    
    if mod(h, 2) == 0
        v = -v;
    end
    
    g = u + v;
end
