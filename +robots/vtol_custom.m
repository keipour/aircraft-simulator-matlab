function m = vtol_custom(add_arm)
    if nargin < 1
        add_arm = false;
    end

    RotorPlacementAngles = [30, 150, 210, 330];
    RotorRotationDirections = [-1, 1, -1, 1];    
    RotorDihedralAngle = 0;
    RotorSidewardAngle = 0; %[45, 0, 0, -45]; 
    RotorInwardAngle = 0; %[-90, 0, 0, -90];

    m = vtol(RotorPlacementAngles, RotorRotationDirections);
    m.SetRotorAngles(RotorInwardAngle, RotorSidewardAngle, RotorDihedralAngle);

    for i = 1 : m.NumOfRotors
        m.Rotors{i}.ArmLength = 1; % in meters
    end
    
    m.PayloadRadius = 0.6;
    
    m.AddServo([1, 4], [0; -1; 0], 0);
    m.AddServo([2, 3], [0; -1; 0], 0);
    
    %m.Servos{1}.SetCurrentAngle(30);
    
    if add_arm
        m.AddEndEffector(arm);
    end
end
