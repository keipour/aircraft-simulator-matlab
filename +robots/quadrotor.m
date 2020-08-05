function m = quadrotor(add_arm)
    if nargin < 1
        add_arm = false;
    end

    RotorPlacementAngles = [45, 135, 225, 315];
    RotorRotationDirections = [-1, 1, -1, 1];
    m = multirotor(RotorPlacementAngles, RotorRotationDirections);
    
    if add_arm
        m.AddEndEffector(arm);
    end
end
