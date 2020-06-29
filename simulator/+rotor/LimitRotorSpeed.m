function [rotor_speed_squared, saturated] = LimitRotorSpeed(rot, rotor_speed_squared)
    flag = false;

    if rotor_speed_squared > rot.MaxrotorSpeedSquared
        rotor_speed_squared = rot.MaxrotorSpeedSquared;
        flag = true;
    end
    if rotor_speed_squared < 0
        rotor_speed_squared = 0;
        flag = true;
    end

    if nargout > 1
        saturated = flag;
    end
end