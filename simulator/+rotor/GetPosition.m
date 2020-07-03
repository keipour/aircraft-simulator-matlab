%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% Copyright (c) 2020 Carnegie Mellon University
% This tool has been developed for educational purposes only as a 
% control tutorial in Air Lab Summer School 2020 (https://theairlab.org). 
% For License information please see the LICENSE file in the root directory.
% Author: Azarakhsh Keipour (keipour [at] cmu.edu)
% Please contact us for any questions or issues.
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function r = GetPosition(rot)
    rx = rot.ArmLength * cosd(rot.DihedralAngle) * cosd(rot.ArmAngle);
    ry = rot.ArmLength * cosd(rot.DihedralAngle) * sind(rot.ArmAngle);
    rz = -rot.ArmLength * sind(rot.DihedralAngle);
    r = [rx; ry; rz];
end
