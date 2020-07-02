%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% Copyright (c) 2020 Carnegie Mellon University
% This tool has been developed for educational purposes only as a 
% control tutorial in Air Lab Summer School 2020 (https://theairlab.org). 
% Azarakhsh Keipour (keipour [at] cmu.edu)
% Please contact us for any questions or issues.
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

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