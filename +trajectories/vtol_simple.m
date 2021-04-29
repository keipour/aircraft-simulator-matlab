% Simple trajectory for a VTOL
function [traj, total_time] = vtol_simple()

    total_time = 15;
    traj = {
        [2, 0, -4, 0], [0, 0]; 
        [400, 10, -100, 0], [90, 90]; 
        };
end
