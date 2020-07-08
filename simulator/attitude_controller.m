classdef attitude_controller < pid_controller

    properties
        RateLimits = [70; 70; 30]; % in deg/s
        OutputMax = [20; 20; 20]; % in rad/s^2
    end
    
    methods

        function euler_accel = CalculateControlCommand(obj, multirotor, rpy_des, dt)
        % Calculates PID response using this formula:
        % P * err + D * -velocity + I * error_integral
            
            % Calculate the error in radians
            rpy_err = wrapToPi(deg2rad(rpy_des - multirotor.State.RPY));
            
            % Calculate the rate in radians
            rpy_dot_err = deg2rad(0 - multirotor.State.EulerRate);
            
            % Update the error integral
            obj.ErrorIntegral = obj.ErrorIntegral + rpy_err * dt;
            
            % Calculate the PID result
            euler_accel = obj.P * rpy_err + ...
                obj.D * rpy_dot_err + obj.I * obj.ErrorIntegral;
            
            % Apply the euler rate limits
            euler_accel = pid_controller.ApplyRateLimits(obj.P, obj.D, rpy_err, ...
                deg2rad(multirotor.State.EulerRate), euler_accel, deg2rad(obj.RateLimits), true);
            
            % Limit the error integral (anti-windup)
            obj.LimitErrorIntegral(euler_accel, rpy_err, rpy_dot_err)
            
            % Limit the output
            euler_accel = obj.LimitOutput(euler_accel);
        end
        
    end
end
