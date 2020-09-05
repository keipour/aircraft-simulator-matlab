classdef collision_box < support_files.collision_base
    %COLLISIONBOX Create a collision geometry as a box primitive
    %   A box primitive is specified by its three side lengths. The box is
    %   axis-aligned with its own body-fixed frame, whose origin is at the
    %   box's center.
    %
    %   BOX = collisionBox(X, Y, Z) creates a box primitive with X, Y, Z as 
    %   its side lengths along the corresponding axes in the geometry-fixed
    %   frame that is ready for collision checking. By default the 
    %   geometry-fixed frame collocates with the world frame.
    %
    %
    %   collisionBox properties:
    %       X           - Side length of the box along x-axis
    %       Y           - Side length of the box along y-axis
    %       Z           - Side length of the box along z-axis
    %       Pose        - Pose of the box relative to the world frame
    %
    %
    %   collisionBox method:
    %       show                - plot box in MATLAB figure
    %
    %   See also checkCollision, collisionCylinder, collisionSphere, collisionMesh.
    
    %   Copyright 2019 The MathWorks, Inc.
    
    properties (Dependent)
        %X Side length of the box along x-axis of the geometry-fixed frame
        X
        
        %Y Side length of the box along y-axis of the geometry-fixed frame
        Y
        
        %Z Side length of the box along z-axis of the geometry-fixed frame
        Z
    end
    
    properties (Access = protected)
        %XInternal
        XInternal
        
        %YInternal
        YInternal
        
        %ZInternal
        ZInternal
    end
    
    methods
        function obj = collision_box(x, y, z)
            %COLLISIONBOX Constructor

            obj.XInternal = x;
            obj.YInternal = y;
            obj.ZInternal = z;
            obj.updateGeometry(x, y, z);

        end

    end
    
    methods
        function set.X(obj, x)
            %set.X

            obj.XInternal = x;
            obj.updateGeometry(x, obj.YInternal, obj.ZInternal);
        end

        function set.Y(obj, y)
            %set.Y

            obj.YInternal = y;
            obj.updateGeometry(obj.XInternal, y, obj.ZInternal);
        end

        function set.Z(obj, z)
            %set.Z

            obj.ZInternal = z;
            obj.updateGeometry(obj.XInternal, obj.YInternal, z);
        end
        
        function x = get.X(obj)
            %get.X
            x = obj.XInternal;
        end
        
        function y = get.Y(obj)
            %get.Y
            y = obj.YInternal;
        end
        
        function z = get.Z(obj)
            %get.Z
            z = obj.ZInternal;
        end
        
        function newObj = copy(obj)
            %copy Creates a deep copy of the collision box object
            newObj = support_files.collision_box(obj.XInternal, obj.YInternal, obj.ZInternal);
            newObj.Pose = obj.Pose;
        end

        
    end
    
    methods (Access = protected)
        function updateGeometry(obj, x, y, z)
            %updateGeometry
            obj.GeometryInternal = robotics.core.internal.CollisionGeometry(x, y, z);
            [F, V] = box_mesh([x,y,z]);
            obj.VisualMeshVertices = V;
            obj.VisualMeshFaces = F;
            obj.EstimatedMaxReach = max([x, y, z]);            
        end
    end
    
    methods(Static, Access = protected)
        function obj = loadobj(objFromMAT)
            %loadobj
            obj = support_files.collision_box(objFromMAT.X, objFromMAT.Y, objFromMAT.Z);
            obj.Pose = objFromMAT.Pose;
            
        end
    end    
end

%% Helper functions

function [F, V] = box_mesh(sz)
    %boxMesh Create mesh for a box shape. The origin is at the
    %   center of the box. The input arguments are the three side
    %   lengths of the box, given as a 3-by-1 vector
    xl = sz(1); yl = sz(2); zl = sz(3);

    V = [xl/2, -yl/2, -zl/2; 
         xl/2,  yl/2, -zl/2;
        -xl/2,  yl/2, -zl/2;
        -xl/2, -yl/2, -zl/2;
         xl/2, -yl/2,  zl/2; 
         xl/2,  yl/2,  zl/2;
        -xl/2,  yl/2,  zl/2;
        -xl/2, -yl/2,  zl/2];
    F = [1 2 6;
         1 6 5;
         2 3 7;
         2 7 6;
         3 4 8;
         3 8 7;
         4 1 5;
         4 5 8;
         5 6 7;
         5 7 8;
         1 4 2;
         2 4 3];

    % Flip between CW and CCW ordering
    F = [F(:,1), F(:,3), F(:,2)];
end
