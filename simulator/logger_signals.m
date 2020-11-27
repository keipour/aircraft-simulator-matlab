classdef logger_signals < handle
    properties (Constant)
        MeasuredStates = 1;
        DesiredEulerAcceleration = 2;
        DesiredLinearAcceleration = 3;
        DesiredRPY = 4;
        DesiredPositionYaw = 5;
        DesiredContactForce = 6; % in the contact frame assuming X is perpendicular
        RotorSpeedsSquaredCommand = 7;

        % Set it to the largest number
        Max = 7;
    end
end

