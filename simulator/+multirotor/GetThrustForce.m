function F = GetThrustForce(obj, RotorSpeedsSquared)
    F = zeros(3, 1);
    for i = 1 : obj.NumOfRotors
       F = F + rotor.GetThrustForce(obj.Rotors{i}, RotorSpeedsSquared(i));
    end
end
