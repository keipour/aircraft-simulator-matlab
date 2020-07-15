classdef last_commands < handle

    properties(Constant)
        DesiredEulerAcceleration support_files.data_field = support_files.data_field;
        DesiredLinearAcceleration support_files.data_field = support_files.data_field;
        DesiredRPY support_files.data_field = support_files.data_field;
        DesiredPositionYaw support_files.data_field = support_files.data_field;
        RotorSpeedsSquaredCommand support_files.data_field = support_files.data_field;
    end
    
    methods(Static)
        function Reset(obj)
            obj.DesiredEulerAcceleration.Reset();
            obj.DesiredLinearAcceleration.Reset();
            obj.DesiredRPY.Reset();
            obj.DesiredPosition.Reset();
            obj.RotorSpeedsSquaredCommand.Reset();
        end
    end
    
end
