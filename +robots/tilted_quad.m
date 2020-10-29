function m = tilted_quad(add_arm)
    if nargin < 1
        add_arm = false;
    end

    RotorPlacementAngles = [0, 90, 180, 270];
    RotorRotationDirections = [-1, 1, -1, 1];    
    RotorDihedralAngle = 0;
    RotorSidewardAngle = [-20, 10, 20, -10]; 
    %RotorSidewardAngle = [-30, 30, -30, 30]; % This is a special interesting case
    RotorInwardAngle = 0;

    m = multirotor(RotorPlacementAngles, RotorRotationDirections);
    m.SetRotorAngles(RotorInwardAngle, RotorSidewardAngle, RotorDihedralAngle);

    
    if add_arm
        m.AddEndEffector(arm);
    end
end
