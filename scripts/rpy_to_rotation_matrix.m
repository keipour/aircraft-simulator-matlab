function obj = rpy_to_rotation_matrix(RPY)
    roll = deg2rad(RPY(1));
    pitch = deg2rad(RPY(2));
    yaw = deg2rad(RPY(3));
    obj.R = angle2dcm(yaw, pitch, roll);
end