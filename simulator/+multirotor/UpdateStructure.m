function obj = UpdateStructure(obj)
    obj.NumOfRotors = length(obj.Rotors);
    obj.I = multirotor.CalcInertia(obj);
    obj.I_inv = pinv(obj.I);
end
