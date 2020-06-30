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
                res{i} = obj.AnalyzeResponse(rpys(:, i), rpy_des(i), signal_names{i});

                % Plot the analysis if asked
                if plot == true
                    subplot(3, 1, i);
                    obj.PlotAnalysis(res{i});
                end
                
                % Print the analysis results
                obj.PrintAnalysis(res{i});
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
        
        function h = PlotSignalsByName(obj, cols, signals, gridon)
            if nargin < 4
                gridon = false;
            end
            if ~iscell(signals)
                signals = {signals};
            end
            
            n_sig = length(signals);
            X = cell(n_sig, 1);
            L = cell(n_sig, 1);
            for i = 1 : n_sig
                [X{i}, L{i}] = obj.StateHistory.GetField(signals{i});
            end
            
            h = obj.PlotSignalsFromData(cols, X, L, gridon);
        end
        
        function h = PlotSignalsFromData(obj, cols, data, labels, gridon)
            if nargin < 5
                gridon = false;
            end
            if nargin < 4
                labels = {};
            end
            
            if ~iscell(data)
                data = {data};
            end
            
            n_data = length(data);
            total_size = 0;
            for i = 1 : n_data
                total_size = total_size + size(data{i}, 2);
            end
            
            t = obj.GetTimeSteps();
            
            h = figure;
            rows = ceil(total_size / cols);
            
            curr_plot = 1;
            for i = 1 : n_data
                for j = 1 : size(data{i}, 2)
                    subplot(rows, cols, curr_plot);
                    plot_signal(t, data{i}(:, j));
                    lbl = {};
                    if length(labels) >= i || length(labels{i}) >= j
                        lbl = labels{i}(:, j);
                    end
                    title(lbl);
                    xlabel('Time (s)');
                    ylabel(lbl);
                    if gridon
                        grid on
                    end
                    fix_plot_limits({t}, {data{i}(:, j)});
                    curr_plot = curr_plot + 1;
                end
            end
        end
        
    end
    
    %% Private methods
    methods(Access=protected)
        function res = AnalyzeResponse(obj, X, des, signal_name)
            res.SignalName = signal_name;
            res.Values = X;
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
        
        function PrintAnalysis(obj, res)
            fprintf('Analysis for response of %s going from %0.2f to %0.2f:\n', ...
                res.SignalName, res.StartValue, res.DesiredValue);
            fprintf('    Total Simulation Time: %0.3f (s)\n', obj.TotalTime);
            fprintf('    Simulation Time Step : %0.2f (ms)\n', obj.TimeStep * 1000);
            fprintf('    The system type is   : %s\n', res.SystemTypeName);
            
            if res.SystemType == 0
                fprintf('\n');
                return;
            end
            
            if res.SystemType == 1 % underdamped
                fprintf('    Overshoot Magnitude  : %0.3f\n', res.OvershootMagnitude);
                fprintf('    Overshoot Percentage : %0.2f %%\n', res.OvershootPercentage * 100);
                fprintf('    %s (0%%-100%%)  : %0.3f (s)\n', res.RiseTimeLabel, res.RiseTime);
            elseif res.SystemType == 2 % overdamped
                fprintf('    %s (10%%-90%%)  : %0.3f (s)\n', res.RiseTimeLabel, res.RiseTime);
            end
            fprintf('    Delay Time (0%%-50%%)  : %0.3f (s)\n', res.DelayTime);
            fprintf('    Settling Time (2%%)   : %0.3f (s)\n', res.SettlingTime);
            fprintf('\n');
        end
        
        function PlotAnalysis(obj, res)
            t = obj.GetTimeSteps();

            % Plot the response
            plot_signal(t, res.Values);

            % Plot the desired value
            plot_signal(t, res.DesiredValue);
            
            if res.SystemType ~= 0
                plot_signal(t, res.StartValue, '-')
                plot_delay(t, res);
                plot_rise(t, res);
                plot_settling_time(t, res);
            end
            
            if res.SystemType == 1
                plot_overshoot(t, res);
            end
            
            title([res.SignalName ' Response Analysis']);
            xlabel('Time (s)');
            ylabel(res.SignalName);
            legend([res.SignalName ' Response'], 'Desired Value');
            grid on
            fix_plot_limits({t}, {res.Values, res.DesiredValue});
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

function plot_signal(t, Y, properties, line_width)
    hold on

    % Check if our input is scalar
    if length(Y) ~= length(t)
        Y = Y * ones(length(t), 1);
    end

    % Plot the signal
    if nargin < 3
        plot(t, Y, 'LineWidth', 2);
    elseif nargin < 4
        plot(t, Y, properties);
    else
        plot(t, Y, properties, 'LineWidth', line_width);
    end
    
    hold off
end
        
function fix_plot_limits(Xs, Ys)
    max_x = -inf;
    min_x = inf;
    for i = 1 : length(Xs)
        max_x = max(max_x, max(Xs{i}));
        min_x = min(min_x, min(Xs{i}));
    end
    max_y = -inf;
    min_y = inf;
    for i = 1 : length(Ys)
        max_y = max(max_y, max(Ys{i}));
        min_y = min(min_y, min(Ys{i}));
    end
    xlim([min_x max_x]);
    ylim([min_y - 0.5, max_y + 0.5]);
end

function plot_dotted_line(x, y)
    plot_signal(x, y, 'm-.');
end

function plot_mark(x, y)
    plot_signal(x, y, 'kx', 3);
end

function add_annotation(x, y, label, center)
    plot_signal(x, y, 'k--', 1.5);
    hold on
    if nargin < 4 || center == false
        text((x(1) + x(2)) / 2, (y(1) + y(2)) / 2, label);
    else
        text((x(1) + x(2)) / 2, (y(1) + y(2)) / 2, label, 'HorizontalAlignment', 'center');
    end
    hold off
end

function plot_overshoot(t, res)
    if res.SystemType == 0
        return;
    end
    ox = [t(res.OvershootIndex), t(res.OvershootIndex)];
    oy = [res.Values(res.OvershootIndex), res.DesiredValue];
    add_annotation(ox, oy, '\leftarrow Overshoot');
end

function plot_delay(t, res)
    index = res.DelayTimeIndex;
    if index == 1 || res.SystemType == 0
        return;
    end
    dx = [t(index), t(index)];
    dy = [res.StartValue, res.Values(index)];
    plot_dotted_line(dx, dy);
    
    dx = [t(1), t(index)];
    dy = [res.Values(index), res.Values(index)];
    add_annotation(dx, dy, {'', 'Delay'}, true);
    
    plot_mark(t(index), res.Values(index))
end

function plot_rise(t, res)
    end_ind = res.RiseTimeEndIndex;
    if end_ind == 1 || res.SystemType == 0
        return;
    end
    
    start_ind = res.RiseTimeStartIndex;
    
    if start_ind > 1
        rx = [t(start_ind), t(start_ind)];
        ry = [res.StartValue, res.Values(end_ind)];
        plot_dotted_line(rx, ry);

        rx = [t(1), t(start_ind)];
        ry = [res.Values(start_ind), res.Values(start_ind)];
        plot_dotted_line(rx, ry);

        plot_mark(t(start_ind), res.Values(start_ind))
    end

    end_value = res.Values(end_ind);
    if res.SystemType == 1
        end_value = 0.7 * end_value + 0.3 * res.Values(res.OvershootIndex);
    else
        rx = [t(1), t(start_ind)];
        ry = [res.Values(end_ind), res.Values(end_ind)];
        plot_dotted_line(rx, ry);
    end
    rx = [t(end_ind), t(end_ind)];
    ry = [res.StartValue, end_value];
    plot_dotted_line(rx, ry);
    plot_mark(t(end_ind), res.Values(end_ind))
    
    rx = [t(start_ind), t(end_ind)];
    ry = [end_value, end_value];
    add_annotation(rx, ry, {'', res.RiseTimeLabel}, true);
end

function plot_settling_time(t, res)
    index = res.SettlingTimeIndex;
    if index == 1 || res.SystemType == 0
        return;
    end

    delta = abs(res.DesiredValue - res.StartValue) * 0.02;
    val1 = res.DesiredValue + delta;
    val2 = res.DesiredValue - delta;
    
    hold on
    fill([t(index), t(end), t(end), t(index)], [val1, val1, val2, val2], ...
        [0.5, 0.7, 1], 'FaceAlpha', 0.25, 'EdgeAlpha', 0.25);
    hold off 
    
    % Draw the vertical dotted line
    sx = [t(index), t(index)];
    sy = [res.StartValue, res.Values(index)];
    plot_dotted_line(sx, sy);
    
    % Draw the horizontal dotted line 
    sx = [t(1), t(index)];
    midval = (res.Values(res.RiseTimeEndIndex) + res.Values(res.DelayTimeIndex)) / 2;
    sy = [midval, midval];
    add_annotation(sx, sy, {'', 'Settling Time'}, true);
    
    plot_mark(t(index), res.Values(index))
end
