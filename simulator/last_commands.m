classdef last_commands < handle

    properties(Constant)
        DesiredEulerAcceleration support_files.data_field = support_files.data_field;
        DesiredLinearAcceleration support_files.data_field = support_files.data_field;
        DesiredRPY support_files.data_field = support_files.data_field;
        DesiredWaypoint support_files.data_field = support_files.data_field;
        DesiredContactForce support_files.data_field = support_files.data_field;
        RotorSpeedsSquaredCommand support_files.data_field = support_files.data_field;
        ContactNormal support_files.data_field = support_files.data_field;
    end
    
    methods (Static)
        function Reset()
            last_commands.DesiredEulerAcceleration.Reset();
            last_commands.DesiredLinearAcceleration.Reset();
            last_commands.DesiredRPY.Reset();
            last_commands.DesiredWaypoint.Reset();
            last_commands.DesiredContactForce.Reset();
            last_commands.RotorSpeedsSquaredCommand.Reset();
            last_commands.ContactNormal.Reset();
        end
    end
    
end
