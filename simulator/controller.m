classdef controller < handle
    properties
        ControlAllocation control_allocation
        AttitudeController attitude_controller
        PositionController position_controller
        HMFCController hmfc_controller
    end
    
    methods
        function obj = controller(multirotor)
            obj.ControlAllocation = control_allocation(multirotor);
            obj.AttitudeController = attitude_controller;
            obj.PositionController = position_controller;
            obj.HMFCController = hmfc_controller;
        end
        
        function rotor_speeds_squared = ControlAcceleration(obj, multirotor, lin_acc_des, euler_acc_des)
            rotor_speeds_squared = obj.ControlAllocation.CalcRotorSpeeds(multirotor, lin_acc_des, euler_acc_des);
        end
        
        function euler_accel = ControlAttitude(obj, multirotor, rpy_des, rpy_dot_des, eul_acc_des, dt)
            euler_accel = obj.AttitudeController.CalculateControlCommand(multirotor, rpy_des, rpy_dot_des, eul_acc_des, dt);
        end
        
        function [lin_accel, rpy_des] = ControlPosition(obj, multirotor, pos_des, yaw_des, vel_des, acc_des, dt)
            lin_accel = obj.PositionController.CalculateControlCommand(multirotor, pos_des, vel_des, acc_des, dt);
            rpy_des = obj.PositionController.CalculateAttitude(lin_accel, yaw_des);
        end
        
        function [lin_accel, rpy_des] = ControlForce(obj, multirotor, force_des, pos_des, yaw_des, vel_des, acc_des, vel_mat, force_constraint, dt)
            motion_lin_accel = obj.PositionController.CalculateControlCommand(multirotor, pos_des, vel_des, acc_des, dt);
            force_lin_accel = obj.HMFCController.CalculateControlCommand(multirotor, force_des, [], [], dt);

            lin_accel = obj.HMFCController.CombineForceAndMotion(force_lin_accel, motion_lin_accel, vel_mat, force_constraint);
            rpy_des = obj.PositionController.CalculateAttitude(lin_accel, yaw_des);
        end
        
        function Reset(obj)
            obj.AttitudeController.Reset();
            obj.PositionController.Reset();
        end
    end
end
