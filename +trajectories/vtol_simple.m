% Simple trajectory for a VTOL
function [traj, total_time] = vtol_simple()

    total_time = 8;
    traj = {
        [2, 0, -4, 0], [0, 0]; 
        [20, 0, -4, 0], [90, 90]; 
        };
end
