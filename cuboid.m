function [X, Y, Z] = cuboid(origin, size)
% CUBOID generates vertices for a 3D cuboid.
% ORIGIN is the [x,y,z] coordinate of one corner.
% SIZE is the [width, depth, height] of the cuboid.

    x = origin(1); y = origin(2); z = origin(3);
    w = size(1); d = size(2); h = size(3);
    
    % Define the 8 vertices of the cuboid
    v = [x, y, z;
         x+w, y, z;
         x+w, y+d, z;
         x, y+d, z;
         x, y, z+h;
         x+w, y, z+h;
         x+w, y+d, z+h;
         x, y+d, z+h];

    % Define the 6 faces by connecting the vertices
    f = [1 2 3 4;
         2 6 7 3;
         4 3 7 8;
         1 5 8 4;
         1 2 6 5;
         5 6 7 8];

    % Generate patch data
    X = v(f',1);
    Y = v(f',2);
    Z = v(f',3);
end
