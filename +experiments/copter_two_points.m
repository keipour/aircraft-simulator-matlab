% Trajectory with two waypoints
function [pos, vel, rpy, omega, traj, total_time] = copter_two_points()

    total_time = 15;

    pos = [0; 0; -4];
    vel = [0; 0; 0];
    rpy = [0; 0; 0];
    omega = [0; 0; 0];

    traj = [2, 2, -4, -4, 0, 0; 2, 6, -3, 7, 0, 30];
end
