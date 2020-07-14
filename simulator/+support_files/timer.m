classdef timer < handle
    properties
        TotalTime = 5;                  % in secs
        PlantRate = 2000;               % in Hertz
        AttitudeControllerRate = 450;   % in Hertz
        PositionControllerRate = 55;    % in Hertz
    end
    
    properties (SetAccess = public, GetAccess = public)
        CurrentTime = 0;                % in secs
    end

    properties (SetAccess = protected, GetAccess = protected)
        TimeStepsTimes
        TimeStepsModules
        CurrentTimeIndex
    end
    
    properties (Constant)
        PosControllerIndex = 1;
        AttControllerIndex = 2;
        PlantIndex = 3;
    end
    
    methods
        function obj = timer()
            obj.Update();
        end
        
        function Update(obj)
            [times, modules] = obj.CreateTimeSteps();
            obj.TimeStepsTimes = [0; times];
            obj.TimeStepsModules = [0; modules];
            obj.CurrentTimeIndex = 1;
        end
        
        function flag = IsFinished(obj)
            flag = obj.CurrentTimeIndex > length(obj.TimeStepsTimes);
        end
        
        function set.TotalTime(obj, value)
            mustBePositive(value);
            obj.TotalTime = value;
            obj.Update();
        end
        
        function set.PlantRate(obj, value)
            mustBePositive(value);
            obj.PlantRate = value;
            obj.Update();
        end
        
        function set.AttitudeControllerRate(obj, value)
            mustBePositive(value);
            obj.AttitudeControllerRate = value;
            obj.Update();
        end
    
        function set.PositionControllerRate(obj, value)
            mustBePositive(value);
            obj.PositionControllerRate = value;
            obj.Update();
        end
        
        %function [time, module] = 
        
%         function value = get.CurrentTime(obj)
%             ind = obj.CurrentTimeIndex;
%             if ind > length(obj.TimeStepsTimes)
%                 ind = length(obj.TimeStepsTimes);
%             end
%             value = obj.TimeStepsTimes(ind);
%         end
    end
    
    methods(Access = protected)
        function [times, modules] = CreateTimeSteps(obj)
            T = cell(3, 1);
            T{obj.PosControllerIndex} = 1 / obj.PositionControllerRate : 1 / obj.PositionControllerRate : obj.TotalTime;
            T{obj.AttControllerIndex} = 1 / obj.AttitudeControllerRate : 1 / obj.AttitudeControllerRate : obj.TotalTime;
            T{obj.PlantIndex} = 1 / obj.PlantRate : 1 / obj.PlantRate : obj.TotalTime;
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
