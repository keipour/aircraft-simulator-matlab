% Simple trajectory for a VTOL
function [pos, vel, rpy, omega, traj, total_time] = vtol_simple()

    total_time = 8;

    pos = [0; 0; -4];
    vel = [0; 0; 0];
    rpy = [0; 0; 0];
    omega = [0; 0; 0];

    traj = {
        [2, 0, -4, 0], [0, 0]; 
        [20, 0, -4, 0], [90, 90]; 
        };
end
