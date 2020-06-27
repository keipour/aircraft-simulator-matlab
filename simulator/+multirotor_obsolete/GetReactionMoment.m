function M = GetReactionMoment(obj, RotorSpeedsSquared)
    M = zeros(3, 1);
    for i = 1 : obj.NumOfRotors
       M = M + rotor.GetReactionMoment(obj.Rotors{i}, RotorSpeedsSquared(i));
    end
end
