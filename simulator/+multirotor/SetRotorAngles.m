function obj = SetRotorAngles(obj, RotorInwardAngles, RotorSidewardAngles, RotorDihedralAngles)
    % Set the angles of the rotors
    % Inputs can be scalar or an array of he same length as 
    % the number of rotors.

    if length(RotorInwardAngles) == 1
        RotorInwardAngles = ones(obj.NumOfRotors, 1) * RotorInwardAngles;
    end
    if length(RotorSidewardAngles) == 1
        RotorSidewardAngles = ones(obj.NumOfRotors, 1) * RotorSidewardAngles;
    end
    if length(RotorDihedralAngles) == 1
        RotorDihedralAngles = ones(obj.NumOfRotors, 1) * RotorDihedralAngles;
    end

    % Assign the values
    for i = 1 : obj.NumOfRotors
        obj.Rotors{i} = rotor.SetInwardAngle(obj.Rotors{i}, RotorInwardAngles(i));
        obj.Rotors{i} = rotor.SetSidewardAngle(obj.Rotors{i}, RotorSidewardAngles(i));
        obj.Rotors{i} = rotor.SetDihedralAngle(obj.Rotors{i}, RotorDihedralAngles(i));
    end

    % Update the structure
    obj = multirotor.UpdateStructure(obj);
end
