classdef timer < handle
    properties
        TotalTime = 5;                  % in secs
        PlantRate = 2000;               % in Hertz
        AttitudeControllerRate = 450;   % in Hertz
        PositionControllerRate = 55;    % in Hertz
        TrajectoryControllerRate = 20;  % in Hertz
    end
    
    properties (SetAccess = protected, GetAccess = public)
        CurrentTime = 0;                % in secs
    end

    properties (SetAccess = protected, GetAccess = protected)
        TimeStepsTimes
        TimeStepsModules
        CurrentTimeIndex
    end
    
    properties (Constant)
        TrajControllerIndex = 4;
        PosControllerIndex = 2;
        AttControllerIndex = 3;
        PlantIndex = 1;
    end
    
    methods
        function obj = timer()
            obj.Reset();
        end
        
        function Reset(obj)
            obj.Update();
        end
        
        function Update(obj)
            [obj.TimeStepsTimes, obj.TimeStepsModules] = obj.CreateTimeSteps();
            obj.CurrentTimeIndex = 0;
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
        
        function [time, module, is_finished] = NextTimeStep(obj)
            obj.CurrentTimeIndex = obj.CurrentTimeIndex + 1;
            is_finished = obj.IsFinished();
            if is_finished
                time = 0;
                module = 0;
                return;
            end
            time = obj.TimeStepsTimes(obj.CurrentTimeIndex);
            module = obj.TimeStepsModules(obj.CurrentTimeIndex);
        end
        
        function value = get.CurrentTime(obj)
            ind = obj.CurrentTimeIndex;
            if ind > length(obj.TimeStepsTimes)
                ind = length(obj.TimeStepsTimes);
            end
            value = obj.TimeStepsTimes(ind);
        end
    end
    
    methods(Access = protected)
        function [times, modules] = CreateTimeSteps(obj)
            T = cell(4, 1);
            T{obj.TrajControllerIndex} = 0 : 1 / obj.TrajectoryControllerRate : obj.TotalTime;
            T{obj.PosControllerIndex} = 0 : 1 / obj.PositionControllerRate : obj.TotalTime;
            T{obj.AttControllerIndex} = 0 : 1 / obj.AttitudeControllerRate : obj.TotalTime;
            T{obj.PlantIndex} = 0 : 1 / obj.PlantRate : obj.TotalTime;
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
