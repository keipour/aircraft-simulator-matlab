classdef simulation
    %SIMULATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        TotalTime = 5;      % in secs
        TimeStep = 1e-3;    % in secs
    end
    
    methods
        function t = GetTimeSteps(obj)
            t = 0 : obj.TimeStep : obj.TotalTime;
        end
    end
end

