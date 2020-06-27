function rot = UpdateStructure(rot)
    rot.R = rotor.CalcRotorationMatrix(rot);
    rot.MaxrotorSpeedSquared = (rot.RPMLimit / 30 * pi).^2;
    rot.Position = rotor.GetPosition(rot);
end
