function rot = SetArmAngle(rot, value)
    rot.ArmAngle = value;
    rot = rotor.UpdateStructure(rot);
end
