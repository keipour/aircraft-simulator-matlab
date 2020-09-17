classdef hmf_controller < handle

    properties
        PositionController position_controller
        ForceController force_controller
    end
    
    methods
        function obj = hmf_controller()
            obj.PositionController = position_controller;
            obj.ForceController = force_controller;
        end
        
        function [lin_accel, rpy_des] = ControlMotionAndForce(obj, mult, ...
                force_des, pos_des, yaw_des, vel_des, acc_des, vel_mat, force_constraint, dt)

            motion_lin_accel = obj.PositionController.CalculateControlCommand(mult, pos_des, vel_des, acc_des, dt);
            force_lin_accel = obj.ForceController.CalculateControlCommand(mult, force_des, [], [], dt);

            lin_accel = hmf_controller.CombineMotionAndForce(force_lin_accel, motion_lin_accel, vel_mat, force_constraint);
            rpy_des = obj.PositionController.CalculateAttitude(lin_accel, yaw_des);
        end
        
        function SetAttitudeStrategy(obj, attitude_strategy)
            obj.PositionController.SetAttitudeStrategy(attitude_strategy);
        end

    end
    
    methods (Static)
        function lin_accel = CombineMotionAndForce(force_lin_accel, motion_lin_accel, vel_mat, force_constraint)
            lin_accel = motion_lin_accel;
            if motion_lin_accel(1) > 0
                lin_accel(1) = force_lin_accel(1);
            end
        end
    end
    

end

