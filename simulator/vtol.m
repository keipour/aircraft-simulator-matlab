classdef vtol < multirotor
    properties
        WingDirections = {[0; 1; 0], [0; -1; 0]};
        WingLengths = [1; 1]; % in meters
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
            wrench(6) = wrench(6) - 0.4 * physics.AirDensity * 0.52 / 2 * norm(obj.State.Velocity(1:2))^2; % Add vertical lift
        end
        
    end
end
