%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% Copyright (c) 2020 Carnegie Mellon University
% This tool has been developed for educational purposes only as a 
% control tutorial in Air Lab Summer School 2020 (https://theairlab.org). 
% For License information please see the LICENSE file in the root directory.
% Author: Azarakhsh Keipour (keipour [at] cmu.edu)
% Please contact us for any questions or issues.
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

classdef position_controller < pid_controller

    properties
        AttitudeType attitude_types = attitude_types.Full;
        RateLimits = [7; 7; 9]; % in m/s
        OutputMax = [1; 1; 40]; % in m/s^2
    end
    
    methods

        function lin_accel = CalculateControlCommand(obj, multirotor, pos_des, dt)
        % Calculates PID response using this formula:
        % P * err + D * -velocity + I * error_integral
            
            % Calculate the PID result
            lin_accel = task2_2_calc_acceleration(obj.P, obj.I, obj.D, ...
                multirotor.State.Position, multirotor.State.Velocity, ...
                pos_des, obj.ErrorIntegral, dt);

            % Apply the velocity limits
            pos_err = pos_des - multirotor.State.Position;
            lin_accel = pid_controller.ApplyRateLimits(obj.P, obj.D, pos_err, ...
                multirotor.State.Velocity, lin_accel, obj.RateLimits, false);

            % Limit the error integral (anti-windup)
            vel_err = 0 - multirotor.State.Velocity;
            obj.LimitErrorIntegral(lin_accel, pos_err, vel_err);
            
            % Limit the output
            lin_accel = obj.LimitOutput(lin_accel);
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
            rpy_des = task2_1_calc_attitude(acc_cmd, yaw_des);
        end
        
        function rpy_des = CalculateZeroTiltAttitude(acc_cmd, yaw_des)
            rpy_des = [0; 0; yaw_des];
        end
    end
end

