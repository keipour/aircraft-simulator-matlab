%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% Copyright (c) 2020 Carnegie Mellon University
% This tool has been developed for educational purposes only as a 
% control tutorial in Air Lab Summer School 2020 (https://theairlab.org). 
% For License information please see the LICENSE file in the root directory.
% Author: Azarakhsh Keipour (keipour [at] cmu.edu)
% Please contact us for any questions or issues.
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function rpy_des = task2_1_calc_attitude(acc_cmd, yaw_des)
% This function calculates the desired attitude from the acceleration
% command and the desired yaw.
%   Inputs:
%       acc_cmd: 3x1 acceleration vector in inertial frame
%       yaw_des: Scalar desired yaw in degrees
%   Output:
%       rpy_des: 3x1 calculated orientation (roll, pitch, yaw) in degrees
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

% The gravity acceleration vector (in NED frame)
gravity = physics.Gravity;

% Initializing the desired roll/pitch/yaw vector
rpy_des = [0; 0; yaw_des];

%% Modify the code below:

% Modify to calculate the correct thrust vector
thrust = zeros(3, 1);

% Modify to calculate the correct desired Z axis
z_axis = [0; 0; 1];

% Modify to calculate the correct desired X axis
x_axis = [1; 0; 0];

% Modify to calculate the correct desired Y axis
y_axis = [0; 1; 0];

% Calculate the roll and pitch from the axes
rpy_des(1) = 0;
rpy_des(2) = 0;

end
