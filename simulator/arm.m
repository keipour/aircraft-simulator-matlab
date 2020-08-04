classdef arm < handle
    properties
        % Fixed Properties
        Length = 0.6;               % in m
        ArmMass = 0.8;              % in Kg
        EndEffectorMass = 0.3       % in Kg
        Direction = [1; 0; 0];      % direction of the arm in body frame
        BasePosition = zeros(3, 1); % Base position in body frame
    end

    properties(SetAccess=protected, GetAccess=public)
        EndEffectorPosition     % Position for end-effector in body frame
        R_BR                    % Rotation matrix for end-effector
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
        
        function UpdateStructure(obj)
            obj.EndEffectorPosition = obj.BasePosition + obj.Length * obj.Direction;
            z_axis = -obj.Direction / norm(obj.Direction); % z axis is into the arm
            x_axis = cross([0; 0; -1], z_axis); % x axis is to the right of end effector
            if norm(x_axis) < 1e-5
                x_axis = cross([-1; 0; 0], z_axis);
            end
            x_axis = x_axis / norm(x_axis);
            y_axis = cross(z_axis, x_axis); % y axis is to the up of end-effector
            obj.R_BR = [x_axis, y_axis, z_axis];
        end
        
    end
end