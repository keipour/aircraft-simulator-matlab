classdef position_controller < pid_controller

    properties(SetAccess=protected, GetAccess=protected)
        RateLimits = [7; 7; 9]; % in m/s
    end
    
    methods

        function lin_accel = CalculateControlCommand(obj, multirotor, pos_des, dt)
        % Calculates PID response using this formula:
        % P * err + D * -velocity + I * error_integral
            
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

%             [lin_accel, obj.ErrorIntegral] = task2_calc_acceleration...
%                 (obj.P, obj.I, obj.D, multirotor.State.Position, ...
%                 multirotor.State.Velocity, pos_des, obj.ErrorIntegral, dt);
        end

    end
    
    methods(Static)

        function rpy_des = CalculateAttitude(acc_cmd, yaw_des)
        % Calculate the desired attitude to achieve the input acceleration
        % The desired attitude aligns the required force direction
        % (acceleration + gravity) with the Z axis. FRD frame is used here.
        
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
        
    end
end

