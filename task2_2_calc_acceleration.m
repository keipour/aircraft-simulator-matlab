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
    
    % Calculate the position error
    pos_err = pos_des - curr_pos;

    % Calculate the velocity error
    vel_err = 0 - curr_vel;

    % Update the error integtal term
    err_int = err_int + pos_err * dt;

    % Calculate the acceleration using this formula:
    % P * pos_err + D * -velocity + I * error_integral
    lin_accel = p * pos_err + d * vel_err + i * err_int;
end