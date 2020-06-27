function rot = SetSidewardAngle(rot, value)
    rot.SidewardAngle = value;
    rot = rotor.UpdateStructure(rot);
end
