classdef timer < handle
    properties
        TotalTime = 5;                  % in secs
        PlantRate = 2000;               % in Hertz
        AttitudeControllerRate = 450;   % in Hertz
        PositionControllerRate = 55;    % in Hertz
        CurrentTime = 0;                % in secs
    end
    
    methods
        function [times, modules] = CreateTimeSteps(obj)
            T = cell(3, 1);
            T{1} = 1 / obj.PositionControllerRate : 1 / obj.PositionControllerRate : obj.TotalTime;
            T{2} = 1 / obj.AttitudeControllerRate : 1 / obj.AttitudeControllerRate : obj.TotalTime;
            T{3} = 1 / obj.PlantRate : 1 / obj.PlantRate : obj.TotalTime;
            [times, modules] = merge_times(T);
        end
    end
end

%% Helper function

function [times, modules] = merge_times(T)
    n = length(T);
    ind = ones(n, 1);
    max_ind = cellfun(@length, T);
    times = zeros(sum(max_ind), 1);
    modules = zeros(sum(max_ind), 1);
    t_ind = 1;
    while any(ind <= max_ind)
        tm = inf(n, 1);
        for i = 1 : n
            if ind(i) <= max_ind(i)
                tm(i) = T{i}(ind(i));
            end
        end
        [times(t_ind), modules(t_ind)] = min(tm);
        ind(modules(t_ind)) = ind(modules(t_ind)) + 1;
        t_ind = t_ind + 1;
    end
end
