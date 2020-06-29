function M = GetReactionMoment(rot, rotor_speed_squared)
    rotor_speed_squared = rotor.LimitRotorSpeed(rot, rotor_speed_squared);
    M = rot.R' * [0; 0; rot.RotationDirection * rot.TorqueConstant ...
        * rotor_speed_squared];
end
