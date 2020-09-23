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
                force_des, pos_des, yaw_des, vel_des, acc_des, contact_normal, vel_mat, force_constraint, dt)

            motion_lin_accel = obj.PositionController.CalculateControlCommand(mult, pos_des, vel_des, acc_des, dt);
            force_lin_accel = obj.ForceController.CalculateControlCommand(mult, force_des, contact_normal, eye(3) - vel_mat, dt);

            lin_accel = hmf_controller.CombineMotionAndForce(force_lin_accel, ...
                motion_lin_accel, contact_normal, vel_mat, force_constraint);
            rpy_des = obj.PositionController.CalculateAttitude(lin_accel, yaw_des);
        end
        
        function SetAttitudeStrategy(obj, attitude_strategy)
            obj.PositionController.SetAttitudeStrategy(attitude_strategy);
        end

    end
    
    methods (Static)
        function lin_accel = CombineMotionAndForce(force_lin_accel, motion_lin_accel, contact_normal, vel_mat, force_constraint)

            % Check if we are still supposed to apply force or we should
            % fly away from the contact
            motion_normal = motion_lin_accel' * contact_normal;
            if motion_normal > 0
                lin_accel = motion_lin_accel;
                return;
            end
            
            % Apply the constraints for the force
            force_lin_accel_con = physics.ApplyContactConstraints(force_lin_accel, ...
                contact_normal, eye(3) - vel_mat, force_constraint, eye(3));

            % Apply the constraints for the motion
            motion_lin_accel_con = physics.ApplyContactConstraints(motion_lin_accel, ...
                contact_normal, vel_mat, -force_constraint, eye(3));

            lin_accel = force_lin_accel_con + motion_lin_accel_con;
        end
    end
    

end

