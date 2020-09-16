classdef hmfc_controller < pid_controller

    properties
        RateLimits = [7; 7; 9]; % in m/s
        OutputMax = [1; 1; 6]; % in m/s^2
    end
    
    methods

        function lin_accel = CalculateControlCommand(obj, multirotor, force_des, ~, ~, time)

            persistent last_force
            if isempty(last_force)
                last_force = multirotor.State.Force;
            end
            
            % Calculate time step
            dt = time - obj.LastTime;
        
            % Calculate the error
            force_err = force_des - multirotor.State.Force;
            
            % Calculate the velocity error
            force_rate_err = 0 - (multirotor.State.Force - last_force);
            
            % Update the error integral
            obj.ErrorIntegral = obj.ErrorIntegral + force_err * dt;
            
            % Calculate the PID result
            force_accel = obj.P * force_err + obj.D * force_rate_err + obj.I * obj.ErrorIntegral;

            % Limit the error integral (anti-windup)
            obj.LimitErrorIntegral(force_accel, force_err, force_rate_err);
            
            % Limit the output
            lin_accel = obj.LimitOutput(multirotor.State.Force / multirotor.TotalMass + force_accel * dt);

            % Update the time of the last call
            obj.LastTime = time;
            
            last_force = multirotor.State.Force;
        end
    end
    
    methods (Static)
        function lin_accel = CombineForceAndMotion(force_lin_accel, motion_lin_accel, vel_mat, force_constraint)
            lin_accel = motion_lin_accel;
            if motion_lin_accel(1) > 0
                lin_accel(1) = force_lin_accel(1);
            end
        end
    end
    
end

