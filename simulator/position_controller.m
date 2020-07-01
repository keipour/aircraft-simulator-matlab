classdef position_controller < pid_controller

    properties(SetAccess=protected, GetAccess=protected)
        RateLimits = [7; 7; 9]; % in m/s
    end
    
    methods

        function lin_accel = CalculateControlCommand(obj, multirotor, pos_des, dt)
        % Calculates PID response using this formula:
        % P * err + D * velocity + I * error_integral
            
            % Calculate the error
            pos_err = pos_des - multirotor.State.Position;
            
            % Calculate the velocity error
            vel_err = 0 - multirotor.State.Velocity;
            
            % Use I term only if the time step is provided for integration
            if (nargin > 3)
                obj.UpdateErrorIntegral(pos_err, dt);
            end
            
            % Calculate the PID result
            lin_accel = obj.P * pos_err + ...
                obj.D * vel_err + obj.I * obj.ErrorIntegral;

            % Apply the velocity limits
            lin_accel = pid_controller.ApplyRateLimits(obj.P, obj.D, pos_err, ...
                multirotor.State.Velocity, lin_accel, obj.RateLimits, false);
        end

    end
    
    methods(Static)

        function rpy_des = CalculateAttitude(multirotor, acc_des, yaw_des, dt)
            %z_axis = norm(acc_des + multirotor.Gr
        end
        
    end
end

