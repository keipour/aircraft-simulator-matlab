classdef controller < handle
    properties
        ControlAllocation control_allocation
        AttitudeController attitude_controller
        PositionController position_controller
    end
    
    methods
        function obj = controller(multirotor)
            obj.ControlAllocation = control_allocation(multirotor);
            obj.AttitudeController = attitude_controller;
            obj.PositionController = position_controller;
        end
        
        function rotor_speeds_squared = ControlAcceleration(obj, multirotor, lin_acc_des, euler_acc_des)
            rotor_speeds_squared = obj.ControlAllocation.CalcRotorSpeeds(multirotor, lin_acc_des, euler_acc_des);
        end
        
        function rotor_speeds_squared = ControlAttitude(obj, multirotor, rpy_des, lin_accel, dt)
            euler_accel = obj.AttitudeController.CalculateControlCommand(multirotor, rpy_des, dt);
            rotor_speeds_squared = obj.ControlAcceleration(multirotor, lin_accel, euler_accel);
        end
        
        function rotor_speeds_squared = ControlPosition(obj, multirotor, pos_des, yaw_des, dt)
            lin_accel = obj.PositionController.CalculateControlCommand(multirotor, pos_des, dt);
            rpy_des = obj.PositionController.CalculateAttitude(lin_accel, yaw_des);
            rotor_speeds_squared = obj.ControlAttitude(multirotor, rpy_des, lin_accel, dt);
        end
        
        function Reset(obj)
            obj.AttitudeController.Reset();
            obj.PositionController.Reset();
        end
    end
end
