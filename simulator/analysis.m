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
                    graphics.PlotResponseAnalysis(res{i});
                end
                
                % Print the analysis results
                graphics.PrintResponseAnalysis(res{i});
            end
        end

        function result = AnalyzeDynamicManipulability(mult, wind_force)
            [accel, accel_omni_radius, contact_point] = analysis.AnalyzeAccelerationDynamicManipulability(mult, wind_force, 3);
            omega_dot = analysis.AnalyzeAngularAccelerationDynamicManipulability(mult, 3);
            result = analyze_plant_structure(mult, accel, omega_dot, accel_omni_radius, contact_point);
            if options.DM_PrintAnalysis
                graphics.PrintDynamicManipulabilityAnalysis(result);
            end
        end
        
        function [accel, omni_radius, contact_point] = AnalyzeAccelerationDynamicManipulability(mult, wind_force, n_steps)
            plot_z_axis_from_zero = options.DM_CrossSectionZFromZero;
            accel = analyze_accelerations(mult, wind_force, n_steps);
            [omni_radius, contact_point] = get_maximum_inscribed_sphere(accel, zeros(3, 1));

            draw_sphere_radius = 0;
            if options.DM_DrawAccelerationOmniSphere 
                draw_sphere_radius = omni_radius;
            end
            
            if options.DM_DrawAccelerationConvexHull
                rotation_center = physics.Gravity + wind_force / mult.TotalMass;
                graphics.DrawConvexHull(accel, 'Dynamic Manipulability - Acceleration', 'a', ...
                    draw_sphere_radius, zeros(3, 1), contact_point, options.DM_DrawPointOfRotationToCenterLine, ...
                    rotation_center, options.DM_DrawPointOfRotationSphere);
            end
            graphics.PlotCrossSections(accel, 'Dynamic Manipulability - Acceleration', 'a', ...
                plot_z_axis_from_zero, ...
                contains(options.DM_DrawAccelerationCrossSections, 'x'), ...
                contains(options.DM_DrawAccelerationCrossSections, 'y'), ...
                contains(options.DM_DrawAccelerationCrossSections, 'z'), ...
                draw_sphere_radius, zeros(3, 1));
            %graphics.PlotLateralThrustDynInv(mult, accel, [8; 9; 10], 'Dynamic Manipulability - Acceleration', 'a');
        end
        
        function omega_dot = AnalyzeAngularAccelerationDynamicManipulability(mult, n_steps)
            omega_dot = analyze_angular_accelerations(mult, n_steps);
            if options.DM_DrawAngularAccelerationConvexHull
                graphics.DrawConvexHull(omega_dot, 'Dynamic Manipulability - Angular Acceleration', '\dot{\omega}');
            end
            graphics.PlotCrossSections(omega_dot, 'Dynamic Manipulability - Angular Acceleration', '\dot{\omega}', ...
                false, contains(options.DM_DrawAngularAccelerationCrossSections, 'x'), ...
                contains(options.DM_DrawAngularAccelerationCrossSections, 'y'), ...
                contains(options.DM_DrawAngularAccelerationCrossSections, 'z'), ...
                0, zeros(3, 1));
        end
    end
end

%% Helper functions
function [sphere_radius, contact_point] = get_maximum_inscribed_sphere(X, center_point)
    sphere_radius = 0;
    contact_point = NaN(3, 1);
    if rank(X, 1e-4) < 3
        return;
    end
    k = convhull(X(:, 1), X(:, 2), X(:, 3), 'Simplify', true);
    if math.InConvexHull(center_point', X)
        k2 = unique(k);
        Y = X(k2, :);
        k2 = convhull(Y(:, 1), Y(:, 2), Y(:, 3), 'Simplify', true);
        [sphere_radius, contact_point] = support_files.point2trimesh...
            ('Faces', k2, 'Vertices', Y, 'QueryPoints', center_point', 'UseSubSurface', false);
        sphere_radius = abs(sphere_radius);
    end
end

function accel = analyze_accelerations(mult, wind_force, n_steps)
    n_rotors = mult.NumOfRotors;
    n_total = n_steps ^ n_rotors;
    accel = zeros(n_total, 3);
    mins = zeros(n_rotors, 1);
    maxs = zeros(n_rotors, 1);
    for i = 1 : n_rotors
        maxs(i) = mult.Rotors{i}.MaxSpeedSquared;
        mins(i) = mult.Rotors{i}.MinSpeedSquared;
    end

    steps = (maxs - mins) ./ (n_steps - 1);

    for i = 1 : n_total
        nextnum = dec2base(i - 1, n_steps, n_rotors) - '0';
        rotor_speeds_squared = mins + nextnum' .* steps;
        accel(i, :) = mult.CalculateAccelerationManipulability(wind_force, rotor_speeds_squared);
    end
end

function omega_dot = analyze_angular_accelerations(mult, n_steps)
    n_rotors = mult.NumOfRotors;
    n_total = n_steps ^ n_rotors;
    omega_dot = zeros(n_total, 3);
    mins = zeros(n_rotors, 1);
    maxs = zeros(n_rotors, 1);
    for i = 1 : n_rotors
        maxs(i) = mult.Rotors{i}.MaxSpeedSquared;
        mins(i) = mult.Rotors{i}.MinSpeedSquared;
    end

    steps = (maxs - mins) ./ (n_steps - 1);

    for i = 1 : n_total
        nextnum = dec2base(i - 1, n_steps, n_rotors) - '0';
        rotor_speeds_squared = mins + nextnum' .* steps;
        omega_dot(i, :) = mult.CalculateAngularAccelerationManipulability(rotor_speeds_squared);
    end
end

function result = analyze_plant_structure(multirotor, accel, omega_dot, accel_omni_radius, contact_point)
    result = [];
    result.TranslationType = detect_dynamic_manipulability_type(accel);
    result.RotationType = detect_dynamic_manipulability_type(omega_dot);
    if isequal(size(accel), size(omega_dot))
        result.ActuationRank = detect_actuation_rank([accel, omega_dot]);
    end

    max_z_accel = -min(accel(:, 3));
    total_accel = norm(multirotor.CalculateAccelerationManipulability(zeros(3, 1), 0, true));
    result.VerticalThrustEfficiency = (max_z_accel + physics.Gravity(3)) / (total_accel + physics.Gravity(3));
    result.VerticalTranslationEfficiency = max_z_accel / total_accel;
    result.MaximumHorizontalAcceleration = sqrt(max(accel(:, 1).^2 + accel(:, 2).^2));
    result.MaximumVerticalAcceleration = -max_z_accel;

    result.MaximumAngularAccelerationX = max(abs(omega_dot(:, 1)));
    result.MaximumAngularAccelerationY = max(abs(omega_dot(:, 2)));
    result.MaximumAngularAccelerationZ = max(abs(omega_dot(:, 3)));
    result.AccelerationOmni = accel_omni_radius;
    result.BoundingOmniAcceleration = contact_point; % The acceleration that is bounding the omni-directional acceleration
end


function type = detect_dynamic_manipulability_type(X)
    x_rank = rank(X);
    if x_rank == 3
        type = 'Fully-Actuated in 3-D';
    elseif x_rank == 2
        type = 'Underactuated in 2-D';
    elseif x_rank == 1
        type = 'Underactuated in 1-D';
    else
        error('Error in dynamic manipulability analysis: translation type not recognized');
    end
end

function type = detect_actuation_rank(X)
    x_rank = rank(X);
    if x_rank == 6
        type = 'Full 6-D actuation';
    else
        type = sprintf('%d-D actuation', x_rank);
    end
end

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
