%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% Copyright (c) 2020 Carnegie Mellon University
% This tool has been developed for educational purposes only as a 
% control tutorial in Air Lab Summer School 2020 (https://theairlab.org). 
% For License information please see the LICENSE file in the root directory.
% Author: Azarakhsh Keipour (keipour [at] cmu.edu)
% Please contact us for any questions or issues.
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function [lin_accel, err_int] = task2_2_calc_acceleration(p, i, d, curr_pos, curr_vel, pos_des, err_int, dt)
% This function calculates the PID output of the given position.
%   Inputs:
%       p        :  3x3 diagonal matrix with coefficients for x, y, and z errors
%       i        :  3x3 diagonal matrix with coefficients for x, y, and z error integrals
%       d        :  3x3 diagonal matrix with coefficients for Vx, Vy, and Vz errors
%       curr_pos :  3x1 vector for the current position
%       curr_vel :  3x1 vector for the current velocity
%       pos_des  :  3x1 vector for the desired (target) position
%       err_int  :  3x1 vector for the current error integrals
%       dt       :  Scalar for the time step
%   Outputs:
%       lin_accel:  3x1 calculated output acceleration command
%       err_int  :  3x1 vector for the updated error integrals
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

    %% Modify the code below:

    % Modify to calculate the position error
    pos_err = 0;

    % Modify to calculate the velocity error (we assume that the desired
    % velocity is zero)
    vel_err = 0;

    % Modify to update the error integtal term
    err_int = err_int + 0;

    % Calculate the linear acceleration using this formula:
    % (P * error) + (D * error derivative) + (I * error integral)
    lin_accel = zeros(3, 1);
end