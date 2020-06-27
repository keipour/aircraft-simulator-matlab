function rot = SetArmLength(rot, value)
    rot.ArmLength = value;
    rot = rotor.UpdateStructure(rot);
end
