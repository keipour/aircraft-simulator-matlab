classdef servo < handle
    properties (SetAccess=protected, GetAccess=public)
        Axes
        Rotors
        RotorNumbers
        CurrentAngle = 0;
    end

    %% Public methods
    methods
        function obj = servo(rbt, rotor_numbers, axes, initial_angle)

            obj.RotorNumbers = rotor_numbers;
            n_rotors = length(rotor_numbers);

            if ~iscell(axes)
                axes = {axes};
            end
            
            obj.Rotors = cell(n_rotors, 1);
            for i = 1 : n_rotors
                obj.Rotors{i} = rbt.Rotors{rotor_numbers(i)};
            end
            if length(axes) == 1
                for i = 1 : n_rotors
                    obj.Axes{i} = axes{1};
                end
            else
                for i = 1 : n_rotors
                    obj.Axes{i} = axes{i};
                end
            end
            obj.CurrentAngle = initial_angle;
        end
        
        function SetCurrentAngle(obj, value)
            obj.UpdateRotorAngles(value - obj.CurrentAngle);
        end

        function UpdateRotorAngles(obj, ang_diff)
            if ang_diff == 0
                return;
            end
            obj.CurrentAngle = obj.CurrentAngle + ang_diff;
            for i = 1 : length(obj.Rotors)
                Rax = axang2rotm([obj.Axes{i}', deg2rad(ang_diff)]);
                obj.Rotors{i}.SetR_BR(Rax * obj.Rotors{i}.R_BR);
            end
        end
    end
end