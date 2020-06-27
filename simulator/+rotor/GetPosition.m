function r = GetPosition(rot)
    rx = rot.ArmLength * cosd(rot.DihedralAngle) * cosd(rot.ArmAngle);
    ry = rot.ArmLength * cosd(rot.DihedralAngle) * sind(rot.ArmAngle);
    rz = -rot.ArmLength * sind(rot.DihedralAngle);
    r = [rx; ry; rz];
end
