function F = GetThrustForce(rot, rotor_speed_squared)
    rotor_speed_squared = min(rotor_speed_squared, rot.MaxrotorSpeedSquared);
    F = rot.R' * [0; 0; -rot.ThrustConstant * rotor_speed_squared];
end
