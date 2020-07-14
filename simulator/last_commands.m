classdef last_commands < handle

    properties(Constant)
        AttitudeController support_files.attitude_controller_command = support_files.attitude_controller_command;
        PositionController support_files.position_controller_command = support_files.position_controller_command;
        ControlAllocation support_files.control_allocation_command = support_files.control_allocation_command;
    end
    
end
