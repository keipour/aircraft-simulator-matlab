classdef attitude_controller < handle
    properties
        P = eye(3);
        I = zeros(3);
        D = eye(3);
    end
    
    properties(SetAccess=protected, GetAccess=protected)
        ErrorIntegral = zeros(3, 1);
        WindupMax = [10; 10; 10];
    end
    
    methods
        function SetPID(obj, p, i, d)
            obj.P = p;
            obj.I = i;
            obj.D = d;
        end
        
        function euler_accel = Control(obj, multirotor, rpy_des, dt)
            rpy_err = deg2rad(rpy_des - multirotor.State.RPY);
            rpy_dot_err = deg2rad(multirotor.State.EulerDerivative);
            if (nargin > 3)
                obj.UpdateErrorIntegral(rpy_err, dt);
            end
            euler_accel = obj.P * rpy_err - ...
                obj.D * rpy_dot_err + obj.I * obj.ErrorIntegral;
        end
        
        function Reset(obj)
            obj.ErrorIntegral = zeros(3, 1);
        end
    end
    
    methods(Access=protected)
        function UpdateErrorIntegral(obj, err, dt)
            obj.ErrorIntegral = obj.ErrorIntegral + err * dt;

            for i = 1 : 3
                if obj.ErrorIntegral(i) > obj.WindupMax(i)
                    obj.ErrorIntegral(i) = obj.WindupMax(i);
                end
            end
        end
    end
end

