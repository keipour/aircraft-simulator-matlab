%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% Copyright (c) 2020 Carnegie Mellon University
% This tool has been developed for educational purposes only as a 
% control tutorial in Air Lab Summer School 2020 (https://theairlab.org). 
% Azarakhsh Keipour (keipour [at] cmu.edu)
% Please contact us for any questions or issues.
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function M = GetReactionMoment(rot, rotor_speed_squared)
    rotor_speed_squared = rotor.LimitRotorSpeed(rot, rotor_speed_squared);
    M = rot.R' * [0; 0; rot.RotationDirection * rot.TorqueConstant ...
        * rotor_speed_squared];
end
