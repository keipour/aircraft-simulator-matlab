classdef force_controller < pid_controller

    properties
        RateLimits = [7; 7; 9]; % in m/s -- not used in this module
        ForceAccelerationMax = [1; 1; 1]; 
        OutputMax = [1; 1; 6]; % in m/s^2
    end
    
    methods

        function lin_accel = CalculateControlCommand(obj, mult, force_des, contact_normal, free_force_mat, time)

            % Find the rotation from the inertial to contact
            rot_ic = vrrotvec2mat(vrrotvec([1; 0; 0], contact_normal));
            
            % Take the current force to the contact frame
            curr_force = rot_ic' * mult.State.Force;

            persistent last_force last_lin_accel
            if isempty(last_force)
                last_force = curr_force;
            end
            if isempty(last_lin_accel)
                last_lin_accel = zeros(3, 1);
            end
            
            % Calculate time step
            dt = time - obj.LastTime;

            % Calculate the error
            force_err = -force_des - curr_force;
            
            % Calculate the difference error
            force_rate_err = 0 - (curr_force - last_force);

            % Don't use the rate if it is larger than a threshold
            force_rate_err(abs(force_rate_err) > 1) = 0;
                
            % Update the error integral
            obj.ErrorIntegral = obj.ErrorIntegral + force_err * dt;
            
            % Calculate the PID result
            force_accel = obj.P * force_err + obj.D * force_rate_err + obj.I * obj.ErrorIntegral;
            force_accel = limit_value(force_accel, obj.ForceAccelerationMax);
            
            % Limit the error integral (anti-windup)
            obj.LimitErrorIntegral(force_accel, force_err, force_rate_err);
            
            % Calculate the output in the contact frame
            lin_accel_c = last_lin_accel + force_accel * dt;
            
            % Convert the relevant parts of output back to the inertial space
            lin_accel = rot_ic * free_force_mat * lin_accel_c;
            lin_accel = obj.LimitOutput(lin_accel);

            % Update the time of the last call
            obj.LastTime = time;
            
            last_force = curr_force;
            last_lin_accel = rot_ic' * lin_accel;
        end
    end
    
end

%% Helper functions

function output = limit_value(output, limits)
    for i = 1 : 3
        if output(i) > limits(i)
             output(i) = limits(i);
        elseif output(i) < -limits(i)
             output(i) = -limits(i);
        end
    end
end
