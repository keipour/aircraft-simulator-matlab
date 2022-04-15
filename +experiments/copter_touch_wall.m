% Trajectory for touching the wall and controlling the force
function [pos, vel, rpy, omega, traj, total_time] = copter_touch_wall()

    total_time = 5;
    
    pos = [12; 10; -3];
    vel = [0; 0; 0];
    rpy = [0; 0; 0];
    omega = [0; 0; 0];
    
    traj = {
            [14, 10, -3, 0]; 
            [14, 10, -3, 0, 5, 0, 0];
            };
end
