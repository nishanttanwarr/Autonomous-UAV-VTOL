classdef Camera < handle
    % Camera - Class representing a camera attached to the drone
    
    properties
        % Camera parameters
        fov = 90            % Field of view in degrees
        aspectRatio = 16/9  % Aspect ratio
        nearClip = 0.1      % Near clipping plane
        farClip = 100       % Far clipping plane
        
        % Camera position and orientation
        position = [0, 0, 0]
        orientation = [0, 0, 0]  % [roll, pitch, yaw]
        
        % Parent object (drone)
        parent
        
        % Camera offset from parent
        offset = [0, 0, 0]
        
        % ArUco detection properties
        detectionRange = 40  % Maximum range for ArUco detection
        detectionFOV = 60    % Field of view for ArUco detection (degrees)
    end
    
    methods
        function obj = Camera(parent)
            % Constructor
            obj.parent = parent;
        end
        
        function update(obj)
            % Update camera position and orientation based on parent
            
            % Get parent rotation matrix
            R = obj.parent.getRotationMatrix();
            
            % Update position
            obj.position = obj.parent.position + (R * obj.offset')';
            
            % Update orientation
            obj.orientation = obj.parent.orientation;
        end
        
        function [viewMatrix, projMatrix] = getViewProjectionMatrices(obj)
            % Get view and projection matrices for rendering
            
            % Update camera
            obj.update();
            
            % Calculate view matrix
            viewMatrix = obj.calculateViewMatrix();
            
            % Calculate projection matrix
            projMatrix = obj.calculateProjectionMatrix();
        end
        
        function viewMatrix = calculateViewMatrix(obj)
            % Calculate view matrix
            
            % Extract Euler angles
            roll = obj.orientation(1);
            pitch = obj.orientation(2);
            yaw = obj.orientation(3);
            
            % Calculate rotation matrix
            Rx = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)];
            Ry = [cos(pitch), 0, sin(pitch); 0, 1, 0; -sin(pitch), 0, cos(pitch)];
            Rz = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
            
            % Combine rotations
            R = Rz * Ry * Rx;
            
            % Calculate view matrix
            viewMatrix = [R, -R * obj.position'; 0, 0, 0, 1];
        end
        
        function projMatrix = calculateProjectionMatrix(obj)
            % Calculate projection matrix
            
            % Convert FOV to radians
            fovRad = obj.fov * pi / 180;
            
            % Calculate projection parameters
            f = 1 / tan(fovRad / 2);
            aspect = obj.aspectRatio;
            near = obj.nearClip;
            far = obj.farClip;
            
            % Calculate projection matrix
            projMatrix = [
                f/aspect, 0, 0, 0;
                0, f, 0, 0;
                0, 0, (far+near)/(near-far), 2*far*near/(near-far);
                0, 0, -1, 0
            ];
        end
        
        function canDetect = canDetectArUco(obj, arUcoPosition)
            % Check if an ArUco marker at the given position can be detected
            
            % Update camera
            obj.update();
            
            % Calculate vector to ArUco marker
            toArUco = arUcoPosition - obj.position;
            distance = norm(toArUco);
            
            % Check if within range
            if distance > obj.detectionRange
                canDetect = false;
                return;
            end
            
            % Calculate forward vector
            R = obj.parent.getRotationMatrix();
            forward = (R * [1, 0, 0]')';
            
            % Calculate angle between forward vector and vector to ArUco
            toArUcoNorm = toArUco / distance;
            cosAngle = dot(forward, toArUcoNorm);
            angle = acos(cosAngle) * 180 / pi;
            
            % Check if within FOV
            canDetect = angle <= obj.detectionFOV / 2;
        end
    end
end
