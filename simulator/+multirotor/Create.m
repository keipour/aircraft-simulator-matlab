function m = Create(ArmAngles, RotationDirections)
    % Constructor for the multirotor class
    % The number of rotors is obtained from the length of the
    % ArmAngles input.
    
    m.Mass = 3.0;                 % in Kg
    m.Gravity = [0; 0; 9.80665];  % in m/s^2
    m.PayloadRadius = 0.15;       % in meters
    
    % Create the array of rotors
    m.NumOfRotors = length(ArmAngles);
    m.Rotors = cell(m.NumOfRotors, 1);
    for i = 1 : m.NumOfRotors
        m.Rotors{i} = rotor.Create();
        m.Rotors{i} = rotor.SetArmAngle(m.Rotors{i}, ArmAngles(i));
        m.Rotors{i}.RotationDirection = RotationDirections(i);
    end

    m.InitialState = state.Create();
    m.State = m.InitialState;
    
    m = multirotor.UpdateStructure(m);
end
