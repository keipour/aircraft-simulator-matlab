function m = octorotor_assymmetric(add_arm)
    if nargin < 1
        add_arm = false;
    end
    
    RotorPlacementAngles = [0, 45, 90, 135, 180, 225, 270, 315];
    RotorRotationDirections = [-1, -1, 1, 1, -1, -1, 1, 1];
    RotorDihedralAngle = 0;
    RotorSidewardAngle = 0;
    RotorInwardAngle = [-90, 0, -90, 0, -90, 0, -90, 0];
    RotorDiameters = [8, 12, 8, 12, 8, 12, 8, 12];
    
    m = multirotor(RotorPlacementAngles, RotorRotationDirections);
    m.SetRotorAngles(RotorInwardAngle, RotorSidewardAngle, RotorDihedralAngle);

    for i = 1 : length(RotorDiameters)
        m.Rotors{i}.Diameter = RotorDiameters(i);
    end
    
    if add_arm
        m.AddEndEffector(arm);
    end
end
