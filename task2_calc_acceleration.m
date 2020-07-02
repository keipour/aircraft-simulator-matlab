function [lin_accel, err_int] = task2_calc_acceleration(p, i, d, curr_pos, curr_vel, pos_des, err_int, dt)
% Calculate the PID response

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