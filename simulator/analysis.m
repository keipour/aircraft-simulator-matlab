%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% Copyright (c) 2020 Carnegie Mellon University
% This tool has been developed for educational purposes only as a 
% control tutorial in Air Lab Summer School 2020 (https://theairlab.org). 
% Azarakhsh Keipour (keipour [at] cmu.edu)
% Please contact us for any questions or issues.
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

classdef analysis

    methods(Static)
        
        function response = AnalyzeResponse(t, X, des, signal_name)
        % Analyses the response of a single signal    
            
            response.SignalName = signal_name;
            response.Values = X;
            response.TotalTime = t(end);
            response.TimeStep = t(2) - t(1);
            response.TimeSteps = t;
            response.StartValue = X(1);
            response.DesiredValue = des;
            tol = 1e-3;

            % Calculate the overshoot
            [response.OvershootMagnitude, response.OvershootPercentage, response.OvershootIndex] = ...
                calc_overshoot(response.StartValue, des, X, tol);
            
            % Detect the type of the system (1=Underdamped, 2=Overdamped)
            [response.SystemType, response.SystemTypeName] = ...
                detect_system_type(response.OvershootMagnitude, des, X, tol);
            
            % Calculate the rise (fall) time
            [response.RiseTime, response.RiseTimeLabel, response.RiseTimeStartIndex, response.RiseTimeEndIndex] = ...
                calc_rise_time(des, X, t, response.SystemType, tol);
            
            % Calculate the delay time
            [response.DelayTime, response.DelayTimeIndex] = ...
                calc_delay_time(des, X, t, response.SystemType, tol);
            
            % Calculate the delay time
            [response.SettlingTime, response.SettlingTimeIndex] = calc_settling_time(des, X, t, response.SystemType, 0.02);
        end

        function res = AnalyzeAndOutputResponse(t, X, des_val, signal_names, plot)
        % Analyze the response then print and plot the results
        % Accepts the N-D trajectory, analyzes, prints and plots the result
            
            % Analysis of the response
            N = size(X, 2);
            res = cell(N, 1);
            for i = 1 : N
                res{i} = analysis.AnalyzeResponse(t, X(:, i), des_val(i), signal_names{i});

                % Plot the analysis if asked
                if plot == true
                    subplot(N, 1, i);
                    graphics.PlotAnalysis(res{i});
                end
                
                % Print the analysis results
                graphics.PrintAnalysis(res{i});
            end
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
