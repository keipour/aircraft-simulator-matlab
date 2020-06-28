classdef attitude_controller < handle
    properties
        P = eye(3);
        I = zeros(3);
        D = eye(3);
    end
    
    properties(SetAccess=protected, GetAccess=protected)
        Integral = 0;
    end
    
    methods
        function SetPID(obj, p, i, d)
            obj.P = p;
            obj.I = i;
            obj.D = d;
        end
        
        function euler_accel = Control(obj, multirotor, rpy_des)
            rpy = multirotor.State.RPY;
            rpy_dot = multirotor.State.EulerDerivative;
            euler_accel = obj.P * deg2rad(rpy_des - rpy) - obj.D * deg2rad(rpy_dot);
        end
    end
    
    methods(Access=protected)

    end
end

