classdef environment < handle

    properties
        AverageWind = [0; 0; 0];  % The wind vector in global frame
    end
    
    properties (SetAccess = private, GetAccess = public)
        Objects = {};
        CollisionModels = {};
    end
    
    methods
        function obj_handle = AddCuboidObject(obj, center, sizes, rpy, color)
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
            o.Texture = {};
            obj_handle = obj.AddObject(o);
        end
        
        function obj_handle = AddGroundPlane(obj, xlimits, ylimits, color)

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
            o.Texture = {};
            obj_handle = obj.AddObject(o);
        end
        
        function obj_handle = AddObject(obj, object)
            obj_handle = length(obj.Objects) + 1;
            obj.Objects{obj_handle} = object;
            obj.CollisionModels{length(obj.CollisionModels)+1} = object.Geometry;
        end
        
        function AddTextureToObject(obj, obj_handle, filename, img_scale, patch_size)

            geom = obj.Objects{obj_handle}.Geometry;
            scaled_img = imresize(imread(filename), img_scale);
            
            if obj.Objects{obj_handle}.Type == 1 % Ground
                p_rows = ceil(geom.X / patch_size);
                p_cols = ceil(geom.Y / patch_size);
                obj.Objects{obj_handle}.Texture.Top = repmat(scaled_img, p_rows, p_cols);
            else
                p_rows = geom.X / patch_size;
                p_cols = geom.Y / patch_size;
                p_hgt = geom.Z / patch_size;

                rotated_img = imrotate(scaled_img, -90);
                obj.Objects{obj_handle}.Texture.Front = repeat_image(rotated_img, p_hgt, p_cols);
                obj.Objects{obj_handle}.Texture.Top = repeat_image(rotated_img, p_cols, p_rows);
                obj.Objects{obj_handle}.Texture.Side = repeat_image(scaled_img, p_hgt, p_rows);
            end
        end
                
        function hfig = Visualize(obj)
            hfig = graphics.VisualizeEnvironment(obj);
        end
        
    end
end

%% Helper functions

function img = repeat_image(img, n, m)
    sz = size(img);
    img = repmat(img, ceil(n), ceil(m), 1);
    img = img(1 : ceil(n*sz(1)), 1 : ceil(m*sz(2)), :);
end