function rot = SetRPMLimit(rot, value)
    rot.RPMLimit = value;
    rot = rotor.UpdateStructure(rot);
end
