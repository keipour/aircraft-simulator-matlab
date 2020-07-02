%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% Copyright (c) 2020 Carnegie Mellon University
% This tool has been developed for educational purposes only as a 
% control tutorial in Air Lab Summer School 2020 (https://theairlab.org). 
% Azarakhsh Keipour (keipour [at] cmu.edu)
% Please contact us for any questions or issues.
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

classdef (Abstract) pid_controller < handle

    properties(SetAccess=protected, GetAccess=public)
        P = eye(3);             % Proportional gain
        I = zeros(3);           % Integral gain
        D = eye(3);             % Derivative gain
        Omega_np = ones(3);     % The desired natural frequency (rad/s)
        Zeta_p = 0.5 * ones(3); % The desired damping coefficient
    end
    
    properties(SetAccess=protected, GetAccess=protected)
        ErrorIntegral = zeros(3, 1);
    end

    properties(Abstract)
        RateLimits
        OutputMax
    end

    methods(Abstract)
        output_command = CalculateControlCommand(obj, multirotor, desired, dt)
    end
    
    methods
        function SetPID(obj, p, i, d)
            obj.P = pid_controller.CheckGain(p);
            obj.I = pid_controller.CheckGain(i);
            obj.D = pid_controller.CheckGain(d);
            
            obj.Omega_np = sqrt(diag(obj.P));
            obj.Zeta_p = 0.5 * diag(obj.D) ./ obj.Omega_np;
        end
        
        function SetFrequencyAndDamping(obj, omega_np, zeta_p)
            obj.Omega_np = diag(pid_controller.CheckGain(omega_np));
            obj.Zeta_p = diag(pid_controller.CheckGain(zeta_p));
            
            obj.P = diag(obj.Omega_np.^2);
            obj.I = zeros(3);
            obj.D = diag(2 * obj.Omega_np .* obj.Zeta_p);
        end
    
        function Reset(obj)
            obj.ErrorIntegral = zeros(3, 1);
        end

    end

    methods(Access=protected)
        function LimitErrorIntegral(obj, output, err, err_dot)
            if obj.I == 0
                return;
            end
            
            for i = 1 : 3
                if output(i) > 1.2 * obj.OutputMax(i)
                     obj.ErrorIntegral(i) = (1.2 * obj.OutputMax(i) - obj.P * err - obj.D * err_dot) / obj.I;
                elseif output(i) < -1.2 * obj.OutputMax(i)
                     obj.ErrorIntegral(i) = (-1.2 * obj.OutputMax(i) - obj.P * err - obj.D * err_dot) / obj.I;
                end
            end
        end
        
        function output = LimitOutput(obj, output)
            for i = 1 : 3
                if output(i) > obj.OutputMax(i)
                     output(i) = obj.OutputMax(i);
                elseif output(i) < -obj.OutputMax(i)
                     output(i) = -obj.OutputMax(i);
                end
            end
        end
    end

    methods(Static, Access=protected)
        function A = CheckGain(a)
        % Check if the gain is a diagonal matrix
        
            % Make sure the gain is non-negative
            mustBeNonnegative(a)
            
            if numel(a) == 1
                A = a * eye(3);
            elseif numel(a) == 3
                A = diag(a);
            elseif size(a, 1) == 3 && size(a, 2) == 3
                A = a;
            else
                error('Input gain should be a scalar, a 3-D vector or a 3x3 matrix');
            end
        end
        
        function res = CheckRateLimits(P, D, Lim, Err)
        % Returns a 3-D vector with 1 if element within rate limits and zero if not
        
            tol = 1e-5;
            res = ones(3, 1);
            for i = 1 : 3
                if abs(D(i, i)) > tol && abs(P(i, i) / D(i, i) * Err(i)) > Lim(i)
                    res(i) = 0;
                end
            end
        end

        function res = ApplyRateLimits(P, D, err, rate, accel, rate_limits, is_angular)
            
            % Calculate the result for the case when the rate has reached 
            % the limits (we basically convert the PID to a P controller 
            % on velocity to reach the maximum velocity in this case)
            rate_err_max = rate_limits - rate;
            if is_angular
                rate_err_max = wrapToPi(rate_err_max);
            end
            accel_lim = D * rate_err_max;
            
            % Check if we are actually going to reach the maximum velocity
            lim_check = pid_controller.CheckRateLimits(P, D, rate_limits, err);
            
            % Chose between the PID response and the maximum velocity
            % response based on if we are hitting the limit or not
            res = accel .* lim_check + accel_lim .* (1 - lim_check);
        end
        
    end
end
