classdef arm < handle
    properties
        % Fixed Properties
        Length = 0.8;               % in m
        ArmMass = 0.1;              % in Kg
        EndEffectorMass = 0.3       % in Kg
        Direction = [1; 0; 0];      % direction of the arm in body frame
        BasePosition = zeros(3, 1); % Base position in body frame
    end

    properties(SetAccess=protected, GetAccess=public)
        EndEffectorPosition     % Position for end-effector in body frame
        R_BE                    % Rotation matrix for end-effector
        TotalMass               % Total mass in Kg
        CollisionModel          % Collision model of the arm
    end
    
    %% Public methods
    methods
        function obj = arm()
            obj.UpdateStructure();
        end
        
        function set.Length(obj, value)
            obj.Length = value;
            obj.UpdateStructure();
        end
        
        function set.Direction(obj, value)
            obj.Direction = value;
            obj.UpdateStructure();
        end
        
        function set.BasePosition(obj, value)
            obj.BasePosition = value;
            obj.UpdateStructure();
        end
        
        function set.ArmMass(obj, value)
            obj.ArmMass = value;
            obj.UpdateStructure();
        end
        
        function set.EndEffectorMass(obj, value)
            obj.EndEffectorMass = value;
            obj.UpdateStructure();
        end
        
        function UpdateStructure(obj)
            obj.EndEffectorPosition = obj.BasePosition + obj.Length * obj.Direction;
            z_axis = -obj.Direction / norm(obj.Direction); % z axis is into the arm
            x_axis = cross([0; 0; -1], z_axis); % x axis is to the right of end effector
            if norm(x_axis) < 1e-5
                x_axis = cross([-1; 0; 0], z_axis);
            end
            x_axis = x_axis / norm(x_axis);
            y_axis = cross(z_axis, x_axis); % y axis is to the up of end-effector
            obj.R_BE = [x_axis, y_axis, z_axis];
            
            obj.TotalMass = obj.ArmMass + obj.EndEffectorMass;
            obj.CollisionModel = obj.CalulateCollisionModel();            
        end
        
        function CopyFrom(obj, ee)
            obj.Length = ee.Length;
            obj.ArmMass = ee.ArmMass;
            obj.EndEffectorMass = ee.EndEffectorMass;
            obj.Direction = ee.Direction;
            obj.BasePosition = ee.BasePosition;
            obj.EndEffectorPosition = ee.EndEffectorPosition;
            obj.R_BE = ee.R_BE;
            obj.CollisionModel = ee.CollisionModel;
        end
        
        function cm = CalulateCollisionModel(obj)
            radius = 0.02; % in meters
            cm = support_files.collision_box(obj.Length, 2 * radius, 2 * radius);

            R = vrrotvec2mat(vrrotvec([1, 0, 0], obj.Direction));
            center = (obj.EndEffectorPosition + obj.BasePosition) / 2;
            T = [R, center; 0, 0, 0, 1];
            cm.Pose = T;
        end
        
    end
end