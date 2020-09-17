classdef environment < handle

    properties
        AverageWind = [0; 0; 0];  % The wind vector in global frame
    end
    
    properties (SetAccess = private, GetAccess = public)
        Objects = {};
        CollisionModels = {};
    end
    
    methods
        function AddCuboidObject(obj, center, sizes, rpy, color)
            if nargin < 5
                color = [];
            end
            if isempty(color)
                color = [0, 0, 0.8];
            end
            o.Color = color;
            
            o.Type = 0;

            bx = support_files.collision_box(sizes(1), sizes(2), sizes(3));
            rot_mat = physics.GetRotationMatrixDegrees(rpy(1), rpy(2), rpy(3))';
            bx.Pose = [rot_mat, center; 0 0 0 1];
            o.Geometry = bx;
            o.Normal = rot_mat * [-1; 0; 0];
            o.Friction = 0.1;
            obj.AddObject(o);
        end
        
        function AddGroundPlane(obj, xlimits, ylimits, color)

            ground_depth = 0.001;

            if nargin < 4
                color = [];
            end
            if isempty(color)
                color = [0.5, 0.85, 0.85];
            end
            o.Color = color;
            
            o.Type = 1;
            
            center = [(xlimits(2) + xlimits(1)) / 2; ...
                (ylimits(2) + ylimits(1)) / 2; ground_depth / 2];
            bx = support_files.collision_box(xlimits(2) - xlimits(1), ...
                ylimits(2) - ylimits(1), ground_depth);
            T = trvec2tform(center');
            bx.Pose = T;
            o.Geometry = bx;
            o.Normal = [0; 0; -1];
            o.Friction = 0.1;
            obj.AddObject(o);
        end
        
        function AddObject(obj, object)
            obj.Objects{length(obj.Objects)+1} = object;
            obj.CollisionModels{length(obj.CollisionModels)+1} = object.Geometry;
        end
    end
end

