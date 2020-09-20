function c = fully_actuated(mult, attitude_strategy)
    if nargin < 2
        attitude_strategy = [];
    end
    if isempty(attitude_strategy)
        attitude_strategy = attitude_strategies.Full;
    end

    c = controller(mult);
    
    c.AttitudeController.SetPID(60, 5, 20);
    c.PositionController.SetPID(3, 0, 7);

    c.HMFController.ForceController.SetPID(1, 0, 3);
    c.HMFController.PositionController.SetPID(5, 0, 7);

    c.SetAttitudeStrategy(attitude_strategy);
end
