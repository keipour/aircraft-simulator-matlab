function M = GetThrustMoment(obj, RotorSpeedsSquared)
    M = zeros(3, 1);
    for i = 1 : obj.NumOfRotors
        r = obj.Rotors{i}.Position;
        F = rotor.GetThrustForce(obj.Rotors{i}, RotorSpeedsSquared(i));
        M = M + cross(r, F);
    end
end
