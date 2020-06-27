function M = GetReactionMoment(rot, rotor_speed_squared)
    rotor_speed_squared = min(rotor_speed_squared, rot.MaxrotorSpeedSquared);
    M = rot.R' * [0; 0; rot.RotationDirection * rot.TorqueConstant ...
        * rotor_speed_squared];
end
