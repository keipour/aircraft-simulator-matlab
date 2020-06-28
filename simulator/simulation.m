classdef simulation < handle
    properties
        TotalTime = 5;      % in secs
        TimeStep = 1e-3;    % in secs
        Multirotor multirotor
        Controller controller
    end
    
    properties(SetAccess=protected, GetAccess=public)
        CurrentTime = 0;    % in secs
        CurrentState
    end
    
    properties(SetAccess=protected, GetAccess=protected)
        InitialMultirotor multirotor
        StateHistory state_collection
    end
    
    %% Methods
    methods
        function obj = simulation(multirotor, controller)
            obj.InitialMultirotor = multirotor;
            obj.Controller = controller;
            obj.Reset();
        end
        
        function t = GetTimeSteps(obj)
            t = 0 : obj.TimeStep : obj.TotalTime;
        end
        
        function Reset(obj)
            obj.Controller.Reset();
            obj.Multirotor = multirotor(0, 1);
            obj.Multirotor.CopyFrom(obj.InitialMultirotor);
            obj.CurrentTime = 0;
            obj.StateHistory = state_collection();
            obj.StateHistory.SetCapacity(length(obj.GetTimeSteps()));
            obj.StateHistory.PushBack(obj.Multirotor.State);
        end
        
        function NextStepPlant(obj, rotor_speeds_squared)
            obj.Multirotor.UpdateState(rotor_speeds_squared, obj.TimeStep);
            obj.StateHistory.PushBack(obj.Multirotor.State);
            obj.CurrentTime = obj.CurrentTime + obj.TimeStep;
        end
        
        function rotor_speeds_squared = ControlAttitude(obj, rpy)
            rotor_speeds_squared = obj.Controller.ControlAttitude(obj.Multirotor, rpy, obj.TimeStep);
        end
        
        function flag = IsLastStep(obj)
            if obj.CurrentTime + obj.TimeStep > obj.TotalTime + 1e-6
                flag = true;
            else
                flag = false;
            end
        end
        
        function traj = GetStateTrajectory(obj)
            traj = obj.StateHistory;
        end
        
        function SimulateAttitudeResponse(obj, rpy_des)
            obj.Reset();
            while true
                u = obj.ControlAttitude(rpy_des);
                obj.NextStepPlant(u);
                if obj.IsLastStep()
                    break;
                end
            end
            rpys = obj.StateHistory.GetRPYs();
            res = obj.AnalyzeResponse(rpys(:, 3), rpy_des(3));
            obj.PrintAnalysis(res, 'yaw');
        end
        
        function set.TotalTime(obj, value)
            obj.TotalTime = value;      % in secs
            obj.Reset();
        end

        function set.TimeStep(obj, value)
            obj.TimeStep = value;      % in secs
            obj.Reset();
        end
    end
    
    methods(Access=protected)
        function res = AnalyzeResponse(obj, X, des)
            res.StartValue = X(1);
            res.DesiredValue = des;
            tol = 1e-5;

            % Calculate the overshoot
            [res.OvershootMagnitude, res.OvershootPercentage] = ...
                calc_overshoot(res.StartValue, des, X, tol);
            
            % Detect the type of the system (1=Underdamped, 2=Overdamped)
            [res.SystemType, res.SystemTypeName] = ...
                detect_system_type(res.OvershootMagnitude, tol);
            
            % Detect the rise time
            [res.RiseTime, res.RiseTimeLabel] = ...
                calc_rise_time(des, X, obj.GetTimeSteps(), res.SystemType, tol);
            
        end
        
        function PrintAnalysis(obj, res, signal_name)
            fprintf('Analysis for response of %s going from %0.2f to %0.2f:\n', signal_name, res.StartValue, res.DesiredValue);
            fprintf('    The system type is  : %s\n', res.SystemTypeName);
            if res.SystemType == 1 % underdamped
                fprintf('    Overshoot Magnitude : %0.3f\n', res.OvershootMagnitude);
                fprintf('    Overshoot Percentage: %0.2f%%\n', res.OvershootPercentage * 100);
                fprintf('    %s (0%%-100%%) : %0.3f (s)\n', res.RiseTimeLabel, res.RiseTime);
            else
                fprintf('    %s (10%%-90%%) : %0.3f (s)\n', res.RiseTimeLabel, res.RiseTime);
            end
            
            fprintf('\n');
        end
        
        function Plot(obj, X, plot_title)
            
        end
    end
end

%% Helper functions
function [mag, perc] = calc_overshoot(start, des, X, tol)
    mag = 0;
    perc = 0;
    min_x = min(X);
    max_x = max(X);

    if max_x - min_x < tol
        return;
    end
    
    if des > start
        mag = (max_x - des);
        perc = (max_x - des) / (des - start);
    else
        mag = (des - min_x);
        perc = (des - min_x) / (start - des);
    end
    
    if mag < tol
        mag = 0;
        perc = 0;
    end
end

function [type, typename] = detect_system_type(overshoot, tol)
    type = 1; % Underdamped
    typename = 'Underdamped';
    if overshoot < tol
        type = 2; % Overdamped
        typename = 'Overdamped';
    end
end

function [rise_time, label] = calc_rise_time(des, X, t, sys_type, tol)
    rise_time = 0;
    label = 'Rise Time';
    
    min_x = min(X);
    max_x = max(X);
    if max_x - min_x < tol
        return;
    end
    
    perc_start = 0;
    perc_end = 1;
    if sys_type == 2 % underdamped
        perc_start = 0.1;
        perc_end = 0.9;
    end
    
    if des - X(1) < 0
        X = -X;
        des = -des;
        label = 'Fall Time';
    end
    dist = des - X(1);
    rise_start_index = find(X > (X(1) + dist * perc_start) - tol, 1);
    rise_end_index = find(X > (X(1) + dist * perc_end) - tol, 1);
    if ~isempty(rise_start_index) && ~isempty(rise_end_index)
        rise_time = t(rise_end_index) - t(rise_start_index);
    end
end