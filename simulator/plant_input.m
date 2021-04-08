classdef plant_input < handle

    properties
        RotorSpeedsSquared  = [];
        ServoAngles         = [];                 % Servo angles (in degrees)
        AileronLeftRate     = 0;
        AileronRightRate    = 0;
        RudderRate          = 0;
        ElevatorRate        = 0;
    end
    
    methods
        function obj = plant_input(n_rotors, n_servos)
            if nargin > 0
                obj.RotorSpeedsSquared = zeros(n_rotors, 1);
            end
            if nargin > 1
                obj.ServoAngles = zeros(n_servos, 1);
            end
        end
        
        function CopyFrom(obj, s)
            obj.RotorSpeedsSquared  = s.RotorSpeedsSquared;
            obj.ServoAngles         = s.ServoAngles;
            obj.AileronLeftRate     = s.AileronLeftRate;
            obj.AileronRightRate    = s.AileronRightRate;
            obj.RudderRate          = s.RudderRate;
            obj.ElevatorRate        = s.ElevatorRate;
        end
        
        function s = struct(obj)
            s.RotorSpeedsSquared  = obj.RotorSpeedsSquared;
            s.ServoAngles         = obj.ServoAngles;
            s.AileronLeftRate     = obj.AileronLeftRate;
            s.AileronRightRate    = obj.AileronRightRate;
            s.RudderRate          = obj.RudderRate;
            s.ElevatorRate        = obj.ElevatorRate;            
        end
    end
end
