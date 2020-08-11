classdef environment < handle
    
    properties
        Objects = {};
    end
    
    methods
        function AddCuboidObject(obj, center, sizes, yaw)
            o.Type = 0;
            bx = collisionBox(sizes(1), sizes(2), sizes(3));
            T = trvec2tform(center');
            matZ = axang2tform([0 0 1 rad2deg(yaw)]);
            bx.Pose = T*matZ;
            o.Geometry = bx;
            obj.AddObject(o);
        end
        
        function AddGroundPlane(obj, xlimits, ylimits)
            ground_depth = 0.2;
            
            o.Type = 1;
            center = [(xlimits(2) + xlimits(1)) / 2; ...
                (ylimits(2) + ylimits(1)) / 2; ground_depth / 2];
            bx = collisionBox(xlimits(2) - xlimits(1), ...
                ylimits(2) - ylimits(1), ground_depth);
            T = trvec2tform(center');
            bx.Pose = T;
            o.Geometry = bx;
            obj.AddObject(o);
        end
        
        function AddObject(obj, object)
            obj.Objects{length(obj.Objects)+1} = object;
        end
    end
end

