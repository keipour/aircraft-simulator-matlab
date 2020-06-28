classdef controller < handle
    properties
        ControlAllocation control_allocation
        AttitudeController attitude_controller
    end
    
    methods
        function obj = controller(multirotor)
            obj.ControlAllocation = control_allocation(multirotor);
            obj.AttitudeController = attitude_controller;
        end
        
        function rotor_speeds_squared = Control(obj, multirotor, pos_des, rpy_des, dt)
            euler_accel = obj.AttitudeController.Control(multirotor, rpy_des, dt);
            rotor_speeds_squared = obj.ControlAllocation.CalcRotorSpeeds(multirotor, [0; 0; -2], euler_accel);
        end
    end
end
