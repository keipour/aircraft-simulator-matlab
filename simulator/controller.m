classdef controller < handle
    properties
        ControlAllocation control_allocation
        AttitudeController attitude_controller
        PositionController position_controller
        HMFController hmf_controller
    end
    
    methods
        function obj = controller(mult)
            obj.ControlAllocation = control_allocation(mult);
            obj.AttitudeController = attitude_controller;
            obj.PositionController = position_controller;
            obj.HMFController = hmf_controller;
        end
        
        function rotor_speeds_squared = ControlAcceleration(obj, mult, lin_acc_des, euler_acc_des)
            rotor_speeds_squared = obj.ControlAllocation.CalcRotorSpeeds(mult, lin_acc_des, euler_acc_des);
        end
        
        function euler_accel = ControlAttitude(obj, mult, rpy_des, rpy_dot_des, eul_acc_des, dt)
            euler_accel = obj.AttitudeController.CalculateControlCommand(mult, rpy_des, rpy_dot_des, eul_acc_des, dt);
        end
        
        function [lin_accel, rpy_des] = ControlPosition(obj, mult, pos_des, yaw_des, vel_des, acc_des, dt)
            lin_accel = obj.PositionController.CalculateControlCommand(mult, pos_des, vel_des, acc_des, dt);
            rpy_des = obj.PositionController.CalculateAttitude(lin_accel, yaw_des);
        end
        
        function [lin_accel, rpy_des] = ControlMotionAndForce(obj, mult, force_des, pos_des, yaw_des, vel_des, acc_des, vel_mat, force_constraint, dt)
            [lin_accel, rpy_des] = obj.HMFController.ControlMotionAndForce(mult, ...
                force_des, pos_des, yaw_des, vel_des, acc_des, vel_mat, force_constraint, dt);
        end
        
        function Reset(obj)
            obj.AttitudeController.Reset();
            obj.PositionController.Reset();
        end
        
        function SetAttitudeStrategy(obj, attitude_strategy)
            obj.PositionController.SetAttitudeStrategy(attitude_strategy);
            obj.HMFController.SetAttitudeStrategy(attitude_strategy);
        end
    end
end
