classdef logger_signals < uint32
    enumeration
        MeasuredStates(1);
        DesiredEulerAcceleration(2);
        DesiredLinearAcceleration(3);
        DesiredRPY(4);
        DesiredPositionYaw(5);
        RotorSpeedsSquaredCommand(6);

        % Set it to the largest number
        Max(6);
    end
end

