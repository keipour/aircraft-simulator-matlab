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
        function obj = simulation(multrotor, controller)
            obj.InitialMultirotor = multrotor;
            obj.Controller = controller;
            obj.Multirotor = multirotor(0, 1);
            obj.Reset();
        end
        
        function t = GetTimeSteps(obj)
            t = 0 : obj.TimeStep : obj.TotalTime;
        end
        
        function Reset(obj)
            obj.Controller.Reset();
            
            % Keep some state fields
            istate = obj.Multirotor.InitialState;
            obj.Multirotor.CopyFrom(obj.InitialMultirotor);
            obj.Multirotor.SetInitialState(istate.Position, istate.Velocity, istate.RPY, istate.Omega);
            
            obj.CurrentTime = 0;
            obj.StateHistory = state_collection(obj.Multirotor.NumOfRotors);
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
        
        function res = SimulateAttitudeResponse(obj, rpy_des, plot)
            % Simulate the response
            obj.Reset();
            while true
                u = obj.ControlAttitude(rpy_des);
                obj.NextStepPlant(u);
                if obj.IsLastStep()
                    break;
                end
            end
            
            % Analysis of the response
            rpys = obj.StateHistory.GetRPYs();
            signal_names = {'Roll', 'Pitch', 'Yaw'};
            res = cell(3, 1);
            for i = 1 : 3
                res{i} = obj.AnalyzeResponse(obj.GetTimeSteps(), rpys(:, i), rpy_des(i), signal_names{i});

                % Plot the analysis if asked
                if plot == true
                    subplot(3, 1, i);
                    graphics.PlotAnalysis(res{i});
                end
                
                % Print the analysis results
                graphics.PrintAnalysis(res{i});
            end
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
    
    %% Private methods
    methods(Access=protected)
        function res = AnalyzeResponse(obj, t, X, des, signal_name)
            res.SignalName = signal_name;
            res.Values = X;
            res.TotalTime = obj.TotalTime;
            res.TimeStep = obj.TimeStep;
            res.TimeSteps = t;
            res.StartValue = X(1);
            res.DesiredValue = des;
            tol = 1e-3;

            % Calculate the overshoot
            [res.OvershootMagnitude, res.OvershootPercentage, res.OvershootIndex] = ...
                calc_overshoot(res.StartValue, des, X, tol);
            
            % Detect the type of the system (1=Underdamped, 2=Overdamped)
            [res.SystemType, res.SystemTypeName] = ...
                detect_system_type(res.OvershootMagnitude, des, X, tol);
            
            % Calculate the rise (fall) time
            [res.RiseTime, res.RiseTimeLabel, res.RiseTimeStartIndex, res.RiseTimeEndIndex] = ...
                calc_rise_time(des, X, obj.GetTimeSteps(), res.SystemType, tol);
            
            % Calculate the delay time
            [res.DelayTime, res.DelayTimeIndex] = ...
                calc_delay_time(des, X, obj.GetTimeSteps(), res.SystemType, tol);
            
            % Calculate the delay time
            [res.SettlingTime, res.SettlingTimeIndex] = calc_settling_time(des, X, obj.GetTimeSteps(), res.SystemType, 0.02);
        end
        
    end
end

%% Helper functions
function [mag, perc, ind] = calc_overshoot(start, des, X, tol)
    mag = 0;
    perc = 0;
    ind = 1;
    
    [min_x, min_ind] = min(X - des);
    [max_x, max_ind] = max(X - des);
    if max_x - min_x < tol
        return;
    end
    
    if des > start
        mag = max_x;
        perc = max_x / (des - start);
        ind = max_ind;
    else
        mag = -min_x;
        perc = -min_x / (start - des);
        ind = min_ind;
    end
    
    if mag < tol
        mag = 0;
        perc = 0;
        ind = 1;
    end
end

function [type, typename] = detect_system_type(overshoot, des, X, tol)
    type = 1; % Underdamped
    typename = 'Underdamped';
    if overshoot < tol
        type = 2; % Overdamped
        typename = 'Overdamped';
    end
    
    min_x = min(X);
    max_x = max(X);
    if max_x - min_x < tol || abs(des - X(1)) < tol
        type = 0; % Undetermined
        typename = 'Not Determined';
    end
end

function [rise_time, label, s_ind, e_ind] = calc_rise_time(des, X, t, sys_type, tol)
    rise_time = 0;
    label = 'Rise Time';
    s_ind = 1;
    e_ind = 1;
    
    if sys_type == 0
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
    s_ind = find(X > (X(1) + dist * perc_start) - tol, 1);
    e_ind = find(X > (X(1) + dist * perc_end) - tol, 1);
    if ~isempty(s_ind) && ~isempty(e_ind)
        rise_time = t(e_ind) - t(s_ind);
    else
        s_ind = 1;
        e_ind = 1;
    end
end

function [delay_time, delay_index] = calc_delay_time(des, X, t, sys_type, tol)
    delay_time = 0;
    delay_index = 1;
    
    if sys_type == 0
        return;
    end
    
    if des - X(1) < 0
        X = -X;
        des = -des;
    end
    
    dist = des - X(1);
    delay_index = find(X > (X(1) + dist * 0.5) - tol, 1);
    if ~isempty(delay_index)
        delay_time = t(delay_index) - t(1);
    else
        delay_index = 1;
    end
end

function [settling_time, settling_index] = calc_settling_time(des, X, t, sys_type, thresh)
    settling_time = 0;
    settling_index = 1;
    
    if sys_type == 0
        return;
    end
    
    delta = abs(des - X(1)) * thresh;

    indices = find(X > des + delta | X < des - delta);
    if isempty(indices)
        return;
    end
    settling_index = max(indices) + 1;
    if settling_index <= length(t)
        settling_time = t(settling_index) - t(1);
    else
        settling_index = 1;
    end
end
