function m = odar(add_arm)
    if nargin < 1
        add_arm = false;
    end

    RotorPlacementAngles = zeros(1, 16);
    RotorRotationDirections = repmat([-1, 1], 1, 8);    
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
    m.Rotors{9}.SetRotorAxis([0; 0; 1]);
    m.Rotors{10}.SetRotorAxis([0; 1; 0]);
    m.Rotors{11}.SetRotorAxis([0; 0; -1]);
    m.Rotors{12}.SetRotorAxis([0; -1; 0]);
    m.Rotors{13}.SetRotorAxis([0; 0; 1]);
    m.Rotors{14}.SetRotorAxis([0; 1; 0]);
    m.Rotors{15}.SetRotorAxis([0; 0; -1]);
    m.Rotors{16}.SetRotorAxis([0; -1; 0]);
    
    long_arm = 2; % in meters
    short_arm = 0.3; % in meters
    front_end = [long_arm / 2; 0; 0];
    back_end = [-long_arm / 2; 0; 0];
    second_end = back_end + long_arm * [-cosd(30); sind(30); 0];
    third_end = second_end + long_arm * [-cosd(20)*cosd(20); -sind(20)*cosd(20); -sind(20)];
    
    m.AddRod(front_end, back_end);
    m.AddRod(back_end, second_end);
    m.AddRod(second_end, third_end);
    for i = 1 : 4
        m.Rotors{i}.SetPosition(front_end - short_arm * m.Rotors{i}.R_BR(:, 3));
        m.AddRod(front_end, front_end - short_arm * m.Rotors{i}.R_BR(:, 3));
    end
    for i = 5 : 8
        m.Rotors{i}.SetPosition(back_end - short_arm * m.Rotors{i}.R_BR(:, 3));
        m.AddRod(back_end, back_end - short_arm * m.Rotors{i}.R_BR(:, 3));
    end
    for i = 9 : 12
        m.Rotors{i}.SetPosition(second_end - short_arm * m.Rotors{i}.R_BR(:, 3));
        m.AddRod(second_end, second_end - short_arm * m.Rotors{i}.R_BR(:, 3));
    end
    for i = 13 : 16
        m.Rotors{i}.SetPosition(third_end - short_arm * m.Rotors{i}.R_BR(:, 3));
        m.AddRod(third_end, third_end - short_arm * m.Rotors{i}.R_BR(:, 3));
    end
    
    MinimumRotorSpeed = 15; % Percentage of the maximum limit
    for i = 1 : m.NumOfRotors
        m.Rotors{i}.LowerSpeedPercentage = MinimumRotorSpeed;
    end
    
    m.Mass = 1;
    
    if add_arm
        m.AddEndEffector(arm);
        m.EndEffector.Length = 2;
    end
end
