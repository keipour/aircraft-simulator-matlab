classdef attitude_controller < pid_controller

    properties(SetAccess=protected, GetAccess=protected)
        RateLimits = [70; 70; 30]; % in deg/s
    end
    
    methods

        function euler_accel = CalculateControlCommand(obj, multirotor, rpy_des, dt)
        % Calculates PID response using this formula:
        % P * err + D * -velocity + I * error_integral
            
            % Calculate the error in radians
            rpy_err = wrapToPi(deg2rad(rpy_des - multirotor.State.RPY));
            
            % Calculate the rate in radians
            rpy_dot_err =  wrapToPi(deg2rad(0 - multirotor.State.EulerRate));
            
            % Use I term only if the time step is provided for integration
            if (nargin > 3)
                obj.UpdateErrorIntegral(rpy_err, dt);
            end
            
            % Calculate the PID result
            euler_accel = obj.P * rpy_err + ...
                obj.D * rpy_dot_err + obj.I * obj.ErrorIntegral;
            
            % Apply the euler rate limits
            euler_accel = pid_controller.ApplyRateLimits(obj.P, obj.D, rpy_err, ...
                deg2rad(multirotor.State.EulerRate), euler_accel, deg2rad(obj.RateLimits), true);
        end
        
    end
end
