classdef position_controller < pid_controller

    properties
        AttitudeType attitude_types = attitude_types.Full;
    end
    
    properties(SetAccess=protected, GetAccess=protected)
        RateLimits = [7; 7; 9]; % in m/s
        OutputMax = [1; 1; 100]; % in m/s^2
    end
    
    methods

        function lin_accel = CalculateControlCommand(obj, multirotor, pos_des, dt)
        % Calculates PID response using this formula:
        % P * err + D * -velocity + I * error_integral
            
            % Calculate the error
            pos_err = pos_des - multirotor.State.Position;
            
            % Calculate the velocity error
            vel_err = 0 - multirotor.State.Velocity;
            
            % Update the error integral
            obj.ErrorIntegral = obj.ErrorIntegral + pos_err * dt;
            
            % Calculate the PID result
            lin_accel = obj.P * pos_err + ...
                obj.D * vel_err + obj.I * obj.ErrorIntegral;

            % Apply the velocity limits
            lin_accel = pid_controller.ApplyRateLimits(obj.P, obj.D, pos_err, ...
                multirotor.State.Velocity, lin_accel, obj.RateLimits, false);

            % Limit the error integral (anti-windup)
            obj.LimitErrorIntegral(lin_accel, pos_err, vel_err);
            
            % Limit the output
            lin_accel = obj.LimitOutput(lin_accel);

%             [lin_accel, obj.ErrorIntegral] = task2_calc_acceleration...
%                 (obj.P, obj.I, obj.D, multirotor.State.Position, ...
%                 multirotor.State.Velocity, pos_des, obj.ErrorIntegral, dt);
        end

        function rpy_des = CalculateAttitude(obj, acc_cmd, yaw_des)
        % Calculate the desired attitude to achieve the input acceleration
        % The desired attitude aligns the required force direction
        % (acceleration - gravity) with the Z axis. FRD frame is used here.
            if obj.AttitudeType == attitude_types.Full
                rpy_des = position_controller.CalculateFullAttitude(acc_cmd, yaw_des);
            elseif obj.AttitudeType == attitude_types.ZeroTilt
                rpy_des = position_controller.CalculateZeroTiltAttitude(acc_cmd, yaw_des);
            end
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
        
        function rpy_des = CalculateZeroTiltAttitude(acc_cmd, yaw_des)
            rpy_des = [0; 0; yaw_des];
        end
    end
end

