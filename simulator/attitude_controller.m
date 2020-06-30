classdef attitude_controller < handle

    properties(SetAccess=protected, GetAccess=public)
        P = eye(3);             % Proportional gain
        I = zeros(3);           % Integral gain
        D = eye(3);             % Derivative gain
        Omega_np = ones(3);     % The desired natural frequency (rad/s)
        Zeta_p = 0.5 * ones(3); % The desired damping coefficient
    end
    
    properties(SetAccess=protected, GetAccess=protected)
        ErrorIntegral = zeros(3, 1);
        WindupMax = [2; 2; 2];
        RateLimits = [70; 70; 30];
    end
    
    methods
        function SetPID(obj, p, i, d)
            obj.P = check_gain(p);
            obj.I = check_gain(i);
            obj.D = check_gain(d);
            
            obj.Omega_np = sqrt(diag(obj.P));
            obj.Zeta_p = 0.5 * diag(obj.D) ./ obj.Omega_np;
        end
        
        function SetFrequencyAndDamping(obj, omega_np, zeta_p)
            obj.Omega_np = diag(check_gain(omega_np));
            obj.Zeta_p = diag(check_gain(zeta_p));
            
            obj.P = diag(obj.Omega_np.^2);
            obj.I = zeros(3);
            obj.D = diag(2 * obj.Omega_np .* obj.Zeta_p);
        end
        
        function euler_accel = Control(obj, multirotor, rpy_des, dt)
            rpy_err = deg2rad(rpy_des - multirotor.State.RPY);
            rpy_dot_err = deg2rad(multirotor.State.EulerRate);
            if (nargin > 3)
                obj.UpdateErrorIntegral(rpy_err, dt);
            end
            euler_accel = obj.P * rpy_err - ...
                obj.D * rpy_dot_err + obj.I * obj.ErrorIntegral;
            
            rpy_dot_err_max = deg2rad(obj.RateLimits - multirotor.State.EulerRate);
            euler_accel_lim = obj.D * rpy_dot_err_max;
            
            lim_check = check_rate_limits(obj.P, obj.D, deg2rad(obj.RateLimits), rpy_err);
            
            euler_accel = euler_accel .* lim_check + euler_accel_lim .* (1 - lim_check);
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

%% Helper functions
function A = check_gain(a)
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