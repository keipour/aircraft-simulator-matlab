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

    if add_arm
        m.AddEndEffector(arm);
    end
end
