function m = vtol(add_arm)
    if nargin < 1
        add_arm = false;
    end

    RotorPlacementAngles = [45, 135, 225, 315];
    RotorRotationDirections = [-1, 1, -1, 1];    
    RotorDihedralAngle = 0;
    RotorSidewardAngle = 0; %[45, 0, 0, -45]; 
    RotorInwardAngle = 0; %[-90, 0, 0, -90];

    m = vtol(RotorPlacementAngles, RotorRotationDirections);
    m.SetRotorAngles(RotorInwardAngle, RotorSidewardAngle, RotorDihedralAngle);

    
    if add_arm
        m.AddEndEffector(arm);
    end
end
