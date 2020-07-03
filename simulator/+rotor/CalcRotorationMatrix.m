%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% Copyright (c) 2020 Carnegie Mellon University
% This tool has been developed for educational purposes only as a 
% control tutorial in Air Lab Summer School 2020 (https://theairlab.org). 
% For License information please see the LICENSE file in the root directory.
% Author: Azarakhsh Keipour (keipour [at] cmu.edu)
% Please contact us for any questions or issues.
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function R = CalcRotorationMatrix(rot)
    if isempty(rot.ArmAngle)
        R = eye(3);
        return;
    end

    mu = deg2rad(rot.ArmAngle);            
    phix = deg2rad(rot.InwardAngle);
    phiy = deg2rad(rot.SidewardAngle);

    rotorZB1 = [0, 1, 0; 
              -1, 0, 0; 
              0, 0, 1];

    rotorZB2 = [cos(mu), sin(mu), 0; 
              -sin(mu), cos(mu), 0;
              0, 0, 1];

    rotorZB = rotorZB2 * rotorZB1;

    rotorXp = [1, 0, 0;
             0, cos(phix), sin(phix);
             0, -sin(phix), cos(phix)];

    rotorYpp = [cos(phiy), 0, -sin(phiy);
              0, 1, 0;
              sin(phiy), 0, cos(phiy)];

    R = rotorYpp * rotorXp * rotorZB;
end
