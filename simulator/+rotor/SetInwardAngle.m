function rot = SetInwardAngle(rot, value)
    rot.InwardAngle = value;
    rot = rotor.UpdateStructure(rot);
end
