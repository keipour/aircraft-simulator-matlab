function m = tilted_hex(add_arm)
    if nargin < 1
        add_arm = false;
    end
    
    RotorPlacementAngles = [30, 90, 150, 210, 270, 330];
    RotorRotationDirections = [-1, 1, -1, 1, -1, 1];
    RotorDihedralAngle = 0;
    RotorSidewardAngle = [-30, 30, -30, 30, -30, 30];
    RotorInwardAngle = 0;

    m = multirotor(RotorPlacementAngles, RotorRotationDirections);
    m.SetRotorAngles(RotorInwardAngle, RotorSidewardAngle, RotorDihedralAngle);

    MinimumRotorSpeed = 15; % Percentage of the maximum limit
    for i = 1 : m.NumOfRotors
        m.Rotors{i}.LowerSpeedPercentage = MinimumRotorSpeed;
    end
    
    if add_arm
        m.AddEndEffector(arm);
    end
end
