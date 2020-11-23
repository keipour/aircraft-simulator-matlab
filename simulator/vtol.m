classdef vtol < multirotor
    properties
        % Add your public properties
    end

    properties (SetAccess=protected, GetAccess=public)
        % Add your properties that can only be set internally
    end
    
    properties (SetAccess=protected, GetAccess=protected)
        % Add your properties that can only be seen internally
    end
    
    %% Public methods
    methods
        function obj = vtol(ArmAngles, RotationDirections)
            obj = obj@multirotor(ArmAngles, RotationDirections);
        end
        
        function wrench = CalcGeneratedWrench(obj, rotor_speeds_squared)
            wrench = CalcGeneratedWrench@multirotor(obj, rotor_speeds_squared);
            wrench(6) = wrench(6) + 0; % Add vertical lift
        end
        
        function ChangeRotorAngles(obj, RotorInwardAngles, RotorSidewardAngles)
            % Set the angles of the rotors
            % Inputs can be scalar or an array of he same length as 
            % the number of rotors.

            if length(RotorInwardAngles) == 1
                RotorInwardAngles = ones(obj.NumOfRotors, 1) * RotorInwardAngles;
            end
            if length(RotorSidewardAngles) == 1
                RotorSidewardAngles = ones(obj.NumOfRotors, 1) * RotorSidewardAngles;
            end
            
            % Assign the values
            for i = 1 : obj.NumOfRotors
                obj.Rotors{i}.InwardAngle = RotorInwardAngles(i);
                obj.Rotors{i}.SidewardAngle = RotorSidewardAngles(i);
            end
            
            obj.InitializeDynamicsMethod();
        end
        
        function ChangeRotorR_BRs(obj, angles)
            
            % Assign the values
            for i = 1 : obj.NumOfRotors
                R_BR = roty(angles(i));
                obj.Rotors{i}.R_BR = R_BR;
            end
            
            obj.InitializeDynamicsMethod();
        end
    
    end
end
