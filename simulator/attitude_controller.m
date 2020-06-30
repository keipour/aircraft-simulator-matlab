classdef attitude_controller < pid_controller

    properties(SetAccess=protected, GetAccess=protected)
        RateLimits = [70; 70; 30];
    end
    
    methods

        function euler_accel = Control(obj, multirotor, rpy_des, dt)
        % Calculates PID response using this formula:
        % P * err + D * velocity + I * error_integral
            
            % Calculate the error in radians
            rpy_err = deg2rad(rpy_des - multirotor.State.RPY);
            
            % Calculate the rate in radians
            rpy_dot_err = deg2rad(multirotor.State.EulerRate);
            
            % Use I term only if the time step is provided for integration
            if (nargin > 3)
                obj.UpdateErrorIntegral(rpy_err, dt);
            end
            
            % Calculate the PID result
            euler_accel = obj.P * rpy_err - ...
                obj.D * rpy_dot_err + obj.I * obj.ErrorIntegral;
            
            % Calculate the result for the case when the rate has reached 
            % the limits (we basically convert the PID to a P controller 
            % on velocity to reach the maximum velocity in this case)
            rpy_dot_err_max = deg2rad(obj.RateLimits - multirotor.State.EulerRate);
            euler_accel_lim = obj.D * rpy_dot_err_max;
            
            % Check if we are actually going to reach the maximum velocity
            lim_check = check_rate_limits(obj.P, obj.D, deg2rad(obj.RateLimits), rpy_err);
            
            % Chose between the PID response and the maximum velocity
            % response based on if we are hitting the limit or not
            euler_accel = euler_accel .* lim_check + euler_accel_lim .* (1 - lim_check);
        end
        
    end
end

%% Helper functions
function res = check_rate_limits(P, D, Lim, Err)
% Returns a 3-D vector with 1 if element within rate limits and zero if not
    tol = 1e-5;
    res = ones(3, 1);
    for i = 1 : 3
        if abs(D(i, i)) > tol && abs(P(i, i) / D(i, i) * Err(i)) > Lim(i)
            res(i) = 0;
        end
    end
end
