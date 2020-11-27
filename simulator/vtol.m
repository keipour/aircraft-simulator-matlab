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
        
    end
end
