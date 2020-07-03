%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% Copyright (c) 2020 Carnegie Mellon University
% This tool has been developed for educational purposes only as a 
% control tutorial in Air Lab Summer School 2020 (https://theairlab.org). 
% For License information please see the LICENSE file in the root directory.
% Author: Azarakhsh Keipour (keipour [at] cmu.edu)
% Please contact us for any questions or issues.
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function rot = UpdateStructure(rot)
    rot.R = rotor.CalcRotorationMatrix(rot);
    rot.MaxrotorSpeedSquared = (rot.RPMLimit / 30 * pi).^2;
    rot.Position = rotor.GetPosition(rot);
end
