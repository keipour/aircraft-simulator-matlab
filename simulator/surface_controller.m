classdef surface_controller < pid_controller
    properties
        % Random numbers for limits
        RateLimits = 5; % in deg/s
        OutputMax = 0.2; % in rad/s^2
    end
    
    properties (SetAccess = private, GetAccess = public)
        AttitudeStrategy attitude_strategies = attitude_strategies.FullTilt;
    end
    
    methods
        function ail_angle = CalculateControlCommand(obj, mult,ail_des,ail_dot_des,time)
        % Calculate PID response using following formula:
        % out = ail_des + 
            if isempty(ail_des)
                ail_des = 0;
            end
            % converting input angle within 2pi
            ail_des = rem((ail_des + pi * 2),2 * pi);
            
            % Calculate time step
            dt = time - obj.LastTime;
            
            % Calculate the error 
            %% NEED SETUP AILERON IN MULT
            ail_err = wrapToPi(deg2rad(ail_des - mult.State.Aileron));
            ail_dot_err = deg2rad(ail_dot_des - mult.State.AileronRate);
            
            % Update the error integral
            obj.ErrorIntegral = obj.ErrorIntegral + ail_err * dt;
            
            % Calculate the PID result
            ail_angle = obj.P * ail_err +...
                        obj.I * obj.ErrorIntegral +...
                        obj.D * ail_dot_err;
                    
            % Apply the aileron rate limits
            ail_angle = pid.controller.ApplyRateLimits(obj.P,obj.D,ail_err,...
                deg2rad(mult.State.AileronRate),ail_angle,deg2rad(obj.RateLimits),true);
            
            % Limit the error integral
            obj.LimitErrorIntegral(ail_angle,ail_err,ail_dot_err)
            
            % Limit the output 
            ail_angle = obj.LimitOutput(ail_angle);
            
            % Update the time of the last call
            obj.LastTime = time;
        end
    end
end