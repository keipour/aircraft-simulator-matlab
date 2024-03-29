classdef attitude_controller < pid_controller

    properties
        RateLimits = [70; 70; 30]; % in deg/s
        OutputMax = [8; 8; 8]; % in rad/s^2
    end
    
    methods

        function euler_accel = CalculateControlCommand(obj, mult, rpy_des, rpy_dot_des, eul_acc_des, time)
        % Calculates PID response using this formula:
        % out = acc_des +  D * vel_err + P * ang_err + I * error_integral
            
            if isempty(rpy_dot_des)
                rpy_dot_des = zeros(3, 1);
            end
            if isempty(eul_acc_des)
                eul_acc_des = zeros(3, 1);
            end
        
            % Calculate time step
            dt = time - obj.LastTime;
        
            % Calculate the error in radians
            rpy_err = wrapToPi(deg2rad(rpy_des - mult.State.RPY));
            
            % Calculate the rate in radians
            rpy_dot_err = deg2rad(rpy_dot_des - mult.State.EulerRate);
            
            % Update the error integral
            obj.ErrorIntegral = obj.ErrorIntegral + rpy_err * dt;
            
            % Calculate the PID result
            euler_accel = eul_acc_des + obj.P * rpy_err + ...
                obj.D * rpy_dot_err + obj.I * obj.ErrorIntegral;
            
            % Apply the euler rate limits
            euler_accel = pid_controller.ApplyRateLimits(obj.P, obj.D, rpy_err, ...
                deg2rad(mult.State.EulerRate), euler_accel, deg2rad(obj.RateLimits), true);
            
            % Limit the error integral (anti-windup)
            obj.LimitErrorIntegral(euler_accel, rpy_err, rpy_dot_err)
            
            % Limit the output
            euler_accel = obj.LimitOutput(euler_accel);
            
            % Update the time of the last call
            obj.LastTime = time;
        end
        
    end
end
