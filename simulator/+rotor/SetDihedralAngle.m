function rot = SetDihedralAngle(rot, value)
    rot.DihedralAngle = value;
    rot = rotor.UpdateStructure(rot);
end
