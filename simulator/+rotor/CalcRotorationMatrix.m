function R_BR = CalcRotorationMatrix(rot)
    if isempty(rot.ArmAngle)
        R = eye(3);
        return;
    end

    rotorZB = rotz(rot.ArmAngle) * rotz(90);

    rotorXp = rotx(rot.InwardAngle);

    rotorYpp = roty(rot.SidewardAngle);

    R_BR = rotorZB * rotorXp * rotorYpp;
end
