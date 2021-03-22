function m = odar(add_arm)
    if nargin < 1
        add_arm = false;
    end

    RotorPlacementAngles = zeros(1, 8);
    RotorRotationDirections = [-1, 1, -1, 1, -1, 1, -1, 1];    
    RotorDihedralAngle = 0;
    RotorSidewardAngle = 0;
    RotorInwardAngle = 0;

    m = multirotor(RotorPlacementAngles, RotorRotationDirections);
    m.SetRotorAngles(RotorInwardAngle, RotorSidewardAngle, RotorDihedralAngle);

    m.Rotors{1}.SetRotorAxis([0; 0; 1]);
    m.Rotors{2}.SetRotorAxis([0; 1; 0]);
    m.Rotors{3}.SetRotorAxis([0; 0; -1]);
    m.Rotors{4}.SetRotorAxis([0; -1; 0]);
    m.Rotors{5}.SetRotorAxis([0; 0; 1]);
    m.Rotors{6}.SetRotorAxis([0; 1; 0]);
    m.Rotors{7}.SetRotorAxis([0; 0; -1]);
    m.Rotors{8}.SetRotorAxis([0; -1; 0]);
    
    long_arm = 1; % in meters
    short_arm = 0.3; % in meters
    front_end = [long_arm; 0; 0];
    back_end = [-long_arm; 0; 0];
    m.AddRod(back_end, front_end);
    for i = 1 : 4
        m.Rotors{i}.SetPosition(front_end - short_arm * m.Rotors{i}.R_BR(:, 3));
        m.AddRod(front_end, front_end - short_arm * m.Rotors{i}.R_BR(:, 3));
    end
    for i = 5 :8
        m.Rotors{i}.SetPosition(back_end - short_arm * m.Rotors{i}.R_BR(:, 3));
        m.AddRod(back_end, back_end - short_arm * m.Rotors{i}.R_BR(:, 3));
    end    

    MinimumRotorSpeed = 15; % Percentage of the maximum limit
    for i = 1 : m.NumOfRotors
        m.Rotors{i}.LowerSpeedPercentage = MinimumRotorSpeed;
    end
    
    if add_arm
        m.AddEndEffector(arm);
        m.EndEffector.Length = 2;
    end
end
