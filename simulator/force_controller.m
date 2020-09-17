classdef force_controller < pid_controller

    properties
        RateLimits = [7; 7; 9]; % in m/s -- not used in this module
        ForceAccelerationMax = [1; 1; 1]; 
        OutputMax = [1; 1; 6]; % in m/s^2
    end
    
    methods

        function lin_accel = CalculateControlCommand(obj, mult, force_des, ~, ~, time)

            persistent last_force
            if isempty(last_force)
                last_force = mult.State.Force;
            end
            
            % Calculate time step
            dt = time - obj.LastTime;
        
            % Calculate the error
            force_err = force_des - mult.State.Force;
            
            % Calculate the difference error
            force_rate_err = 0 - (mult.State.Force - last_force);
            % Don't use the rate if it is larger than a threshold
            force_rate_err(abs(force_rate_err) > 1) = 0;
                
            % Update the error integral
            obj.ErrorIntegral = obj.ErrorIntegral + force_err * dt;
            
            % Calculate the PID result
            force_accel = obj.P * force_err + obj.D * force_rate_err + obj.I * obj.ErrorIntegral;
            force_accel = limit_value(force_accel, obj.ForceAccelerationMax);
            
            % Limit the error integral (anti-windup)
            obj.LimitErrorIntegral(force_accel, force_err, force_rate_err);
            
            % Limit the output
            lin_accel = obj.LimitOutput(mult.State.Force / mult.TotalMass + force_accel * dt);

            % Update the time of the last call
            obj.LastTime = time;
            
            last_force = mult.State.Force;
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
