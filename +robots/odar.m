% From paper "ODAR: Aerial Manipulation Platform Enabling Omnidirectional
% Wrench Generation"

function m = odar(add_arm)
    if nargin < 1
        add_arm = false;
    end

    RotorPlacementAngles = zeros(1, 8);
    rot_dirs = [-1, 1, 1, 1];
    RotorRotationDirections = [rot_dirs, -rot_dirs];
    RotorDihedralAngle = 0;
    RotorSidewardAngle = 0;
    RotorInwardAngle = 0;

    m = multirotor(RotorPlacementAngles, RotorRotationDirections);
    m.SetRotorAngles(RotorInwardAngle, RotorSidewardAngle, RotorDihedralAngle);

    m.Rotors{1}.SetRotorAxis([0.68; 0.28; 0.68]);
    m.Rotors{2}.SetRotorAxis([0.68; 0.28; -0.68]);
    m.Rotors{3}.SetRotorAxis([0.68; -0.28; 0.68]);
    m.Rotors{4}.SetRotorAxis([0.68; -0.28; -0.68]);
    
    long_arm = 0.4; % in meters
    short_arm = 0.24; % in meters
    front_end = [long_arm; 0; 0];
    back_end = [-long_arm; 0; 0];
    m.AddRod(back_end, front_end);

    short_arm_cos45 = short_arm * cosd(45);
    m.Rotors{1}.SetPosition(front_end + [0; -short_arm_cos45; short_arm_cos45]);
    m.Rotors{2}.SetPosition(front_end + [0; short_arm_cos45; short_arm_cos45]);
    m.Rotors{3}.SetPosition(front_end + [0; -short_arm_cos45; -short_arm_cos45]);
    m.Rotors{4}.SetPosition(front_end + [0; short_arm_cos45; -short_arm_cos45]);
    
    for i = 5 : 8
        m.Rotors{i}.SetRotorAxis(m.Rotors{i - 4}.R_BR(:, 3));
        m.Rotors{i}.SetPosition(-m.Rotors{i - 4}.Position);
        m.AddRod(front_end, m.Rotors{i - 4}.Position);
        m.AddRod(back_end, m.Rotors{i}.Position);
    end

    % Make the rotors bi-directional and smaller
    for i = 1 : m.NumOfRotors
        m.Rotors{i}.LowerSpeedPercentage = -100; % Percentage of the minimum speed
        m.Rotors{i}.Diameter = 10; % in inch (just for visualization)
    end
    
    m.Mass = 2.6; % in Kg
    
    if add_arm
        m.AddEndEffector(arm);
        m.EndEffector.Length = 2;
    end
end
