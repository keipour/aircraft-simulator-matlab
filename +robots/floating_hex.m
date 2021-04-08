function m = floating_hex(add_arm)
    if nargin < 1
        add_arm = false;
    end

    RotorPlacementAngles = [0, 0, 90, 180, 180, 270];
    RotorRotationDirections = [1, 1, 1, 1, 1, 1];
    RotorDihedralAngle = [90, 0, 0, -90, 0, 0];
    RotorSidewardAngle = [90, 0, 90, 90, 180, 90];
    RotorInwardAngle = [0, 0, 0, 0, 0, 0];

    m = multirotor(RotorPlacementAngles, RotorRotationDirections);
    m.SetRotorAngles(RotorInwardAngle, RotorSidewardAngle, RotorDihedralAngle);    

    MinimumRotorSpeed = -100; % Percentage of the minimum limit to make rotors bidirectional
    for i = 1 : m.NumOfRotors
        m.Rotors{i}.LowerSpeedPercentage = MinimumRotorSpeed;
    end
    
    m.Mass = 2; % in Kg
    
    if add_arm
        m.AddEndEffector(arm);
    end
end
