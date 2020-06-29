function F = GetThrustForce(rot, rotor_speed_squared)
    rotor_speed_squared = rotor.LimitRotorSpeed(rot, rotor_speed_squared);
    F = rot.R' * [0; 0; -rot.ThrustConstant * rotor_speed_squared];
end
