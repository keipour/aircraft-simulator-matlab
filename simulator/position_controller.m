classdef position_controller < pid_controller

    properties
        RateLimits = [7; 7; 9]; % in m/s
        OutputMax = [1; 1; 6]; % in m/s^2
    end
    
    properties (SetAccess = private, GetAccess = public)
        AttitudeStrategy attitude_strategies = attitude_strategies.Full;
    end
    
    methods

        function lin_accel = CalculateControlCommand(obj, mult, pos_des, vel_des, acc_des, time)
        % Calculates PID response using this formula:
        % out = acc_des +  D * vel_err + P * ang_err + I * error_integral
            
            if isempty(vel_des)
                vel_des = zeros(3, 1);
            end
            if isempty(acc_des)
                acc_des = zeros(3, 1);
            end
        
            % Calculate time step
            dt = time - obj.LastTime;
        
            % Calculate the error
            pos_err = pos_des - mult.State.Position;
            
            % Calculate the velocity error
            vel_err = vel_des - mult.State.Velocity;
            
            % Update the error integral
            obj.ErrorIntegral = obj.ErrorIntegral + pos_err * dt;
            
            % Calculate the PID result
            lin_accel = acc_des + obj.P * pos_err + ...
                obj.D * vel_err + obj.I * obj.ErrorIntegral;

            % Apply the velocity limits
            lin_accel = pid_controller.ApplyRateLimits(obj.P, obj.D, pos_err, ...
                mult.State.Velocity, lin_accel, obj.RateLimits, false);

            % Limit the error integral (anti-windup)
            obj.LimitErrorIntegral(lin_accel, pos_err, vel_err);
            
            % Limit the output
            lin_accel = obj.LimitOutput(lin_accel);

            % Update the time of the last call
            obj.LastTime = time;
        end

        function rpy_des = CalculateAttitude(obj, acc_cmd, yaw_des)
        % Calculate the desired attitude to achieve the input acceleration
        % The desired attitude aligns the required force direction
        % (acceleration - gravity) with the Z axis. FRD frame is used here.
            if obj.AttitudeStrategy == attitude_strategies.Full
                rpy_des = position_controller.CalculateFullAttitude(acc_cmd, yaw_des);
            elseif obj.AttitudeStrategy == attitude_strategies.ZeroTilt
                rpy_des = position_controller.CalculateZeroTiltAttitude(acc_cmd, yaw_des);
            end
        end

        function SetAttitudeStrategy(obj, attitude_strategy)
            if ~isa(attitude_strategy, 'attitude_strategies')
                error('Position Controller: Input attitude strategy must be from attitude_strategies enum.');
            end
            obj.AttitudeStrategy = attitude_strategy;
        end

    end
    
    methods(Static)

        function rpy_des = CalculateFullAttitude(acc_cmd, yaw_des)
            %rpy_des = task2_calc_attitude(acc_cmd, yaw_des);
            
            % Initialize the output
            rpy_des = [0; 0; yaw_des];
            
            % Find the desired Z axis
            force = acc_cmd - physics.Gravity;
            z_axis = -force / norm(force);
            
            % Find the desired X axis
            x_c = cross([-sind(yaw_des); cosd(yaw_des); 0], z_axis);
            x_axis = x_c / norm(x_c);
            
            % Find the desired Y axis
            y_axis = cross(z_axis, x_axis);
            
            % Calculate the roll and pitch
            rpy_des(1) = atan2d(y_axis(3), z_axis(3));
            rpy_des(2) = -asind(x_axis(3));
        end
        
        function rpy_des = CalculateZeroTiltAttitude(~, yaw_des)
            rpy_des = [0; 0; yaw_des];
        end
    end
end

