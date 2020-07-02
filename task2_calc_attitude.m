function rpy_des = task2_calc_attitude(acc_cmd, yaw_des)
% This function calculates the desired attitude from the acceleration
% command and the desired yaw.
%   Inputs:
%       acc_cmd: 3x1 acceleration vector in inertial frame
%       yaw_des: Scalar desired yaw in degrees
%   Output:
%       rpy_des: 3x1 calculated attitude (roll, pitch, yaw) in degrees
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

gravity = physics.Gravity;
rpy_des = [0; 0; yaw_des];

%% Modify the code below:

% Find the thrust vector
thrust = acc_cmd + gravity;

% Find the desired Z axis
z_axis = thrust / norm(thrust);

% Find the desired X axis
y_c = cross([-sind(yaw_des); cosd(yaw_des); 0], z_axis);
x_axis = y_c / norm(y_c);

% Find the desired Y axis
y_axis = cross(z_axis, x_axis);

% Calculate the roll and pitch
rpy_des(1) = atan2d(y_axis(3), z_axis(3));
rpy_des(2) = -asind(x_axis(3));

end
