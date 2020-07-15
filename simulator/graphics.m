classdef graphics

    methods(Static)
        function PlotResponseAnalysis(response)
            t = response.TimeSteps;

            % Plot the response
            plot_signal(t, response.Values);

            % Plot the desired value
            plot_signal(t, response.DesiredValue);
            
            if response.SystemType ~= 0
                plot_signal(t, response.StartValue, '-')
                plot_delay(t, response);
                plot_rise(t, response);
                plot_settling_time(t, response);
            end
            
            if response.SystemType == 1
                plot_overshoot(t, response);
            end
            
            title([response.SignalName ' Response Analysis']);
            xlabel('Time (s)');
            ylabel(response.SignalName);
            legend([response.SignalName ' Response'], 'Desired Value');
            grid on
            fix_plot_limits({t}, {response.Values, response.DesiredValue});
        end
        
        function h = PlotSignalsByName(cols, signals, gridon)
            if nargin < 4
                gridon = false;
            end
            if ~iscell(signals)
                signals = {signals};
            end
            
            n_sig = length(signals);
            X = cell(n_sig, 1);
            T = cell(n_sig, 1);
            L = cell(n_sig, 1);
            for i = 1 : n_sig
                [X{i}, T{i}, L{i}] = logger.GetField(signals{i});
            end
            
            h = graphics.PlotSignalsFromData(cols, X, T, L, gridon);
        end
        
        function h = PlotSignalsFromData(cols, data, times, labels, gridon)
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
                if iscell(data{i})
                    total_size = total_size + size(data{i}{1}, 2);
                else
                    total_size = total_size + size(data{i}, 2);
                end
            end
            
            h = figure;
            rows = ceil(total_size / cols);
            
            curr_plot = 1;
            for i = 1 : n_data
                n_datacols = size(data{i}, 2);
                if iscell(data{i})
                    n_datacols = size(data{i}{1}, 2);
                end
                for j = 1 : n_datacols
                    subplot(rows, cols, curr_plot);
                    if iscell(data{i})
                        plot_signal(times{i}{1}, data{i}{1}(:, j));
                        plot_signal(times{i}{2}, data{i}{2}(:, j));
                        legend('Measured', 'Commanded');
                    else
                        plot_signal(times{i}, data{i}(:, j));
                    end
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
                    if iscell(data{i})
                        fix_plot_limits(times{i}, {data{i}{1}(:, j), data{i}{2}(:, j)});
                    else
                        fix_plot_limits({times{i}}, {data{i}(:, j)});
                    end
                    curr_plot = curr_plot + 1;
                end
            end
        end
        
        function PrintResponseAnalysis(res)
            fprintf('Analysis for response of %s going from %0.2f to %0.2f:\n', ...
                res.SignalName, res.StartValue, res.DesiredValue);
            fprintf('    Total Simulation Time: %0.3f (s)\n', res.TotalTime);
            fprintf('    Simulation Time Step : %0.2f (ms)\n', res.TimeStep * 1000);
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

        function VisualizeMultirotor(multirotor, axes_only)
            visualize_multirotor(multirotor, axes_only);
        end
        
        function PrintDynamicManipulabilityAnalysis(res)
            fprintf('Analysis for the multirotor structure:\n');
            fprintf('    Translation Actuation Type           : %s\n', res.TranslationType);
            fprintf('    Vertical Thrust Efficiency           : %0.2f %%\n', res.VerticalThrustEfficiency * 100);
            fprintf('    Vertical Translation Efficiency      : %0.2f %%\n', res.VerticalTranslationEfficiency * 100);
            fprintf('    Maximum Vertical Acceleration        : %0.2f m/s^2\n', res.MaximumVerticalAcceleration);
            fprintf('    Maximum Horizontal Acceleration      : %0.2f m/s^2\n', res.MaximumHorizontalAcceleration);
            fprintf('    Maximum Angular Acceleration Around X: %0.2f rad/s^2\n', res.MaximumAngularAccelerationX);
            fprintf('    Maximum Angular Acceleration Around Y: %0.2f rad/s^2\n', res.MaximumAngularAccelerationY);
            fprintf('    Maximum Angular Acceleration Around Z: %0.2f rad/s^2\n', res.MaximumAngularAccelerationZ);
        end
        
        function DrawConvexHull(X, plot_title, label)
            figure;
            r = rank(X);
            if r == 3
                k = convhull(X(:, 1), X(:, 2), X(:, 3), 'Simplify', true);
                trisurf(k, X(:, 1), X(:, 2), X(:, 3), 'FaceColor','cyan', 'LineStyle', '-');
            elseif r == 2
                X_xy = rotate_3d_plane_to_xy(X);
                k = convhull(X_xy(:, 1), X_xy(:, 2), 'Simplify', true);
                fill3(X(k, 1), X(k, 2), X(k, 3), 'c');
            elseif r == 1
                plot3(X(:, 1), X(:, 2), X(:, 3), 'c');
            end
            xlabel(['$' label '_x$'], 'Interpreter', 'latex');
            ylabel(['$' label '_y$'], 'Interpreter', 'latex');
            zlabel(['$' label '_z$'], 'Interpreter', 'latex');
            axis equal
            title(plot_title);
        end
        
        function PlotCrossSections(X, plot_title, label)
            if rank(X) < 3
                return;
            end
            plot_cross_section(X, plot_title, label, 'x');
            plot_cross_section(X, plot_title, label, 'y');
            plot_cross_section(X, plot_title, label, 'z');

            % % Another method based on dynamic inversion - Kept just for
            % % double proof when needed
            %
            % ca = control_allocation(multirotor);
            % figure;
            % sgtitle(plot_title);
            % Zq = linspace(min(X(:, 3)), 0, 9);
            % X_xy = X(:, 1:2);
            % xylimits = [min(X_xy(:)), max(X_xy(:))];
            % for i = 1 : 9
            %     subplot(3, 3, i);
            %     is_angular = true;
            %     [xc, yc] = get_cross_section_from_dynamic_inversion ...
            %         (X(:, 1), X(:, 2), Zq(i), ca, multirotor, is_angular);
            %     plot(xc, yc);
            %     xlabel(['$' label '_x$'], 'Interpreter', 'latex');
            %     ylabel(['$' label '_y$'], 'Interpreter', 'latex');
            %     title(['$' label '_z = ' num2str(Zq(i), '%0.2f') '$'], 'Interpreter', 'latex');
            %     xlim(xylimits);
            %     ylim(xylimits);
            %     axis equal
            % end
        end
    end
end

%% Helper functions
function plot_cross_section(X, plot_title, label, pivot_axis)
    if lower(pivot_axis) == 'z'
        axis_labels = {'x', 'y', 'z'};
        Pivot_q = linspace(min(X(:, 3)), 0, 9);
        axis1 = X(:, 1);
        axis2 = X(:, 2);
        axis3 = X(:, 3);
    elseif lower(pivot_axis) == 'y'
        axis_labels = {'x', 'z', 'y'};
        Pivot_q = linspace(min(X(:, 2)), max(X(:, 2)), 9);
        axis1 = X(:, 1);
        axis2 = X(:, 3);
        axis3 = X(:, 2);
    elseif lower(pivot_axis) == 'x'
        axis_labels = {'y', 'z', 'x'};
        Pivot_q = linspace(min(X(:, 1)), max(X(:, 1)), 9);
        axis1 = X(:, 2);
        axis2 = X(:, 3);
        axis3 = X(:, 1);
    end
    figure;
    sgtitle([plot_title ' (' upper(axis_labels{3}) ' axis)']);
    X_xy = [axis1 axis2];
    xylimits = [min(X_xy(:)), max(X_xy(:))];
    for i = 1 : 9
        subplot(3, 3, i);
        [xc, yc] = get_cross_section(axis1, axis2, axis3, Pivot_q(i));
        plot(xc, yc);
        xlabel(['$' label '_' axis_labels{1} '$'], 'Interpreter', 'latex');
        ylabel(['$' label '_' axis_labels{2} '$'], 'Interpreter', 'latex');
        title(['$' label '_' axis_labels{3} ' = ' num2str(Pivot_q(i), '%0.2f') '$'], 'Interpreter', 'latex');
        xlim(xylimits);
        ylim(xylimits);
        axis equal
    end
end

function [x, y] = get_cross_section(X, Y, Z, z)
    Nq = 1e5;
    minx = min(X);
    miny = min(Y);
    maxx = max(X);
    maxy = max(Y);
    
    Xq = rand(Nq, 1) * (maxx - minx) + minx;
    Yq = rand(Nq, 1) * (maxy - miny) + miny;
    
    in1 = inhull([Xq, Yq, z*ones(Nq, 1)], [X, Y, Z]);
    x = Xq(in1);
    y = Yq(in1);
end 

function [x, y] = get_cross_section_from_dynamic_inversion(X, Y, z, ca, m, is_angluar)
    if nargin < 6
        is_angluar = false;
    end
    Nq = 5*1e3;
    minx = min(X);
    miny = min(Y);
    maxx = max(X);
    maxy = max(Y);
    
    Xq = rand(Nq, 1) * (maxx - minx) + minx;
    Yq = rand(Nq, 1) * (maxy - miny) + miny;
    
    out1 = false(Nq, 1);
    for i = 1 : Nq
        if is_angluar
            [~, out1(i)] = ca.CalcRotorSpeeds(m, zeros(3, 1), [Xq(i); Yq(i); z]);
        else
            [~, out1(i)] = ca.CalcRotorSpeeds(m, [Xq(i); Yq(i); z], zeros(3, 1));
        end
    end
    x = Xq(~out1);
    y = Yq(~out1);
end

function X_xy = rotate_3d_plane_to_xy(X)
    x_mean = mean(X);
    [~, ~, W] = svd(X - x_mean, 0);
    normal = W(:,end);
    axis_angle_rot = vrrotvec(normal, [0; 0; 1]);
    rot_mat = vrrotvec2mat(axis_angle_rot);
    X_plane = (rot_mat * (X - x_mean)')';
    X_xy = [X_plane(:, 1), X_plane(:, 2)];
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
        end_value = 0.9 * end_value + 0.1 * res.StartValue;
    else
        rx = [t(1), t(start_ind)];
        ry = [res.Values(end_ind), res.Values(end_ind)];
        plot_dotted_line(rx, ry);
    end
    rx = [t(end_ind), t(end_ind)];
    ry = [res.StartValue, res.Values(end_ind)];
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
    add_annotation(sx, sy, {'', 'Settling Time (2%)'}, true);
    
    plot_mark(t(index), res.Values(index))
end

%% Visualizer for multirotors
% This file visualizes the multirotor geometry
% Author: Azarakhsh Keipour (keipour@gmail.com)
% Last updated: June 22, 2020
function visualize_multirotor(m, plot_axes_only)
    % Create the new figure
    figure;

    % Visualization settings
    box_height = 0.1; % in meters
    axis_arrow_size = 0.3; % in meters
    plot_title = 'Your Cool Multirotor';
    arm_labels_on = true;
    motor_size = 0.02; % in meters -- set to zero if don't want motors
    lighting_on = true; % turn on the special lighting
    rotor_diameter = 12; % in inches

    % Initialization
    num_of_rotors = m.NumOfRotors;

    payload_size = m.PayloadRadius;
    arm_lengths = cell2mat(cellfun(@(s)s.ArmLength', m.Rotors, 'uni', 0));
    arm_angles = cell2mat(cellfun(@(s)s.ArmAngle', m.Rotors, 'uni', 0)) * pi / 180;
    phi_dihs = cell2mat(cellfun(@(s)s.DihedralAngle', m.Rotors, 'uni', 0)) * pi / 180;
    [~, arms_order] = sort(arm_angles);
    
    % Calculate the rotor coordinates (which also serve as the end points for arms)
    X_rotors = arm_lengths .* cos(phi_dihs) .* cos(arm_angles);
    Y_rotors = arm_lengths .* cos(phi_dihs) .* sin(arm_angles);
    Z_rotors = arm_lengths .* sin(-phi_dihs);
    
    % Visualize the geometry

    % Draw the arms
    for i = 1 : num_of_rotors
        hold on
        plotArm([X_rotors(i); Y_rotors(i); Z_rotors(i)], m.Rotors{i}.R_BR * [0;0;-1], i, arm_labels_on, plot_axes_only, motor_size);
    end

    % Draw the rotors
    for i = 1 : num_of_rotors
        hold on
        if plot_axes_only == false
            plotRotor([X_rotors(i); Y_rotors(i); Z_rotors(i)], m.Rotors{i}.R_BR * [0;0;-1], ...
                m.Rotors{i}.RotationDirection, axis_arrow_size, motor_size, rotor_diameter);
        else
            plotAxes([X_rotors(i); Y_rotors(i); Z_rotors(i)], m.Rotors{i}.R_BR,  axis_arrow_size / 2);
        end
    end

    % Draw the central payload box
    hold on
    if plot_axes_only == false
        plotBox(X_rotors, Y_rotors, Z_rotors, arm_lengths, arms_order, payload_size, box_height);
    else
        [sx, sy, sz] = sphere;
        sphere_size = axis_arrow_size / 10;
        sx = sx * sphere_size;
        sy = sy * sphere_size;
        sz = sz * sphere_size;
        surf(sx, sy, sz);
        plotAxes(zeros(3, 1), eye(3),  axis_arrow_size / 2);
    end

    % Make the plot more presentable

    % Rotate the axes for better visualization
    set(gca, 'Xdir', 'reverse')
    set(gca, 'Zdir', 'reverse')

    % Equalize the axes scales
    axis equal;

    % Add title and axis labels
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title(plot_title);


    % Change lighting
    if lighting_on
        camlight
        lighting gouraud %phong
    end
end

function plotBox(X_rotors, Y_rotors, Z_rotors, arm_lengths, arms_order, box_size, box_height)
    num_of_rotors = length(arms_order);
    box_xs = (X_rotors * box_size) ./ arm_lengths;
    box_ys = (Y_rotors * box_size) ./ arm_lengths;
    box_zs = (Z_rotors * box_size) ./ arm_lengths;
    fill3(box_xs(arms_order), box_ys(arms_order), box_zs(arms_order) + box_height / 2, 'cyan');
    fill3(box_xs(arms_order), box_ys(arms_order), box_zs(arms_order) - box_height / 2, 'cyan');
    for i = 1 : num_of_rotors
        j = mod(i, num_of_rotors) + 1;
        xs = [box_xs(arms_order(i)) box_xs(arms_order(i)) box_xs(arms_order(j)) box_xs(arms_order(j))];
        ys = [box_ys(arms_order(i)) box_ys(arms_order(i)) box_ys(arms_order(j)) box_ys(arms_order(j))];
        zs = [(box_zs(arms_order(i)) + box_height / 2) (box_zs(arms_order(i)) - box_height / 2) (box_zs(arms_order(j)) - box_height / 2) (box_zs(arms_order(j)) + box_height / 2)];
        fill3(xs, ys, zs, 'cyan');
    end
end

function plotArm(position, z_axis, num, arm_labels_on, plot_axes_only, motor_size)
    line_width = 3;
    if plot_axes_only
        line_width = 1;
    end
    plot3([0, position(1)], [0, position(2)], [0, position(3)], 'k', 'LineWidth', line_width);
    if arm_labels_on
        label_dist = 0.02;
        dp = -(motor_size + label_dist) * z_axis;
        if plot_axes_only
            dp = -dp;
        end
        text(position(1) + dp(1), position(2) + dp(2), position(3) + dp(3), num2str(num), 'Interpreter', 'none');
    end
end

function plotRotor(position, axis, direction, arrow_size, motor_size, rotor_diameter)
    rotor_size = rotor_diameter * 0.0254 / 2; 
    rotor_color = [0.4, 0.4, 1]; % CW
    if direction == 1
        rotor_color = [0.4, 1, 0.4];
    end
    motor_color = 'black';
    dmot = motor_size * axis;
    pos_m1 = position - dmot;
    pos_m2 = position + dmot;
    plot3([pos_m1(1), pos_m2(1)], [pos_m1(2), pos_m2(2)], [pos_m1(3), pos_m2(3)], 'Color', motor_color, 'LineWidth', 10);
    circlePlane3D(pos_m2, axis, rotor_size, 0.005, 1, rotor_color, arrow_size, direction);
end

function plotAxes(position, Rotation, arrow_size)
    colors = {'green', 'blue', 'red'};
    labels = {'$\hat{x}$', '$\hat{y}$', '$\hat{z}$'};
    label_dist = 0.02;
    for i = 1 : 3
        end_pos = position + arrow_size*Rotation(:, i);
        label_pos = end_pos + label_dist * Rotation(:, i);
        arrow3d([position(1) end_pos(1)], [position(2) end_pos(2)], [position(3) end_pos(3)], 0.8, 0.005, 0.01, colors{i});
        text(label_pos(1), label_pos(2), label_pos(3), labels{i}, 'Interpreter', 'latex');
    end
end

%% Draw a 3-D circle
% Downloaded from https://www.mathworks.com/matlabcentral/fileexchange/37879-circle-plane-in-3d
% With some modifications and bug fixes
function H = circlePlane3D( center, normal, radious, theintv, normalon, color, arrow_size, direction)
    %CIRCLEPLANE3D Summary of this function goes here
    %--------------------------------------------------------------------------
    %Generate a circle plane in 3D with the given center and radious
    %The plane is defined by the normal vector
    %theintv is the interval theta which allow you to control your polygon
    %shape
    % Example:,
    %
    %   circlePlane3D([0 0 0], [1 -1 2], 5, 0.2, 1, [0 0 1], '-'); 
    %   circlePlane3D([3 3 -3],[0 1 1], 3, 0.1, 1, 'y', '-');
    %   
    %   Cheng-Yuan Wu <ieda_wind@hotmail.com>
    %   Version 1.00
    %   Aug, 2012
    %--------------------------------------------------------------------------

    % The disc plotting has a bug when the normal is completely in Z direction
    if normal(1) == 0 && normal(2) == 0 
        normal(1) = normal(1) + 1e-8;
    end
    %generate circle polygon
    t = 0:theintv:2*pi;
    x = radious*cos(t);
    y = radious*sin(t);
    z = zeros(size(x));
    %compute rotate theta and axis
    zaxis = [0 0 1];
    normal = normal/norm(normal);
    ang = acos(dot(zaxis,normal));
    axis = cross(zaxis, normal)/norm(cross(zaxis, normal));
    % A skew symmetric representation of the normalized axis 
    axis_skewed = [ 0 -axis(3) axis(2) ; axis(3) 0 -axis(1) ; -axis(2) axis(1) 0]; 
    % Rodrigues formula for the rotation matrix 
    R = eye(3) + sin(ang)*axis_skewed + (1-cos(ang))*axis_skewed*axis_skewed;
    fx = R(1,1)*x + R(1,2)*y + R(1,3)*z;
    fy = R(2,1)*x + R(2,2)*y + R(2,3)*z;
    fz = R(3,1)*x + R(3,2)*y + R(3,3)*z;
    %translate center
    fx = fx+center(1);
    fy = fy+center(2);
    fz = fz+center(3);
    H = fill3(fx, fy, fz, color);
    if normalon == 1
        hold on;
        normal_scaled = normal * arrow_size;
        %plot3([center(1) center(1)+normal(1)],[center(2) center(2)+normal(2)],[center(3) center(3)+normal(3)],'-');
        H = arrow3d([center(1) center(1)+normal_scaled(1)],[center(2) center(2)+normal_scaled(2)],[center(3) center(3)+normal_scaled(3)], 0.8, 0.01, 0.02, 'red');
    end
    
    % draw the rotations with arrows
    raise_amount = 0.1;
    arc_length = 0.75; % * 2pi
    n_points = floor(length(t) * arc_length);
    X = (fx(1 : n_points) + center(1)) / 2 + raise_amount * normal(1);
    Y = (fy(1 : n_points) + center(2)) / 2 + raise_amount * normal(2);
    Z = (fz(1 : n_points) + center(3)) / 2 + raise_amount * normal(3);
    line(X,Y,Z, 'LineWidth', 2, 'Color', 'magenta');
    if direction == 1
        arrow3d([X(end-50) X(end)], [Y(end-50) Y(end)], [Z(end-50) Z(end)], 0, 0.005, 0.01, 'red');
        %quiver3(X(end-1), Y(end-1), Z(end-1), X(end)-X(end-1), Y(end)-Y(end-1), Z(end)-Z(end-1),1, 'LineWidth', 10, 'Color', 'magenta');
    else
        arrow3d([X(50) X(1)], [Y(50) Y(1)], [Z(50) Z(1)], 0, 0.005, 0.01, 'red');
        %quiver3(X(2), Y(2), Z(2), X(1)-X(2), Y(1)-Y(2), Z(1)-Z(2), 1, 'LineWidth', 10, 'Color', 'magenta');
    end
end

%% Draw a 3-D arrow
function [h]=arrow3d(x,y,z,head_frac,radii,radii2,colr)
%
% The function plotting 3-dimensional arrow
%
% h=arrow3d(x,y,z,head_frac,radii,radii2,colr)
%
% The inputs are:
%       x,y,z =  vectors of the starting point and the ending point of the
%           arrow, e.g.:  x=[x_start, x_end]; y=[y_start, y_end];z=[z_start,z_end];
%       head_frac = fraction of the arrow length where the head should  start
%       radii = radius of the arrow
%       radii2 = radius of the arrow head (defult = radii*2)
%       colr =   color of the arrow, can be string of the color name, or RGB vector  (default='blue')
%
% The output is the handle of the surfaceplot graphics object.
% The settings of the plot can changed using: set(h, 'PropertyName', PropertyValue)
%
% example #1:
%        arrow3d([0 0],[0 0],[0 6],.5,3,4,[1 0 .5]);
% example #2:
%        arrow3d([2 0],[5 0],[0 -6],.2,3,5,'r');
% example #3:
%        h = arrow3d([1 0],[0 1],[-2 3],.8,3);
%        set(h,'facecolor',[1 0 0])
% 
% Written by Moshe Lindner , Bar-Ilan University, Israel.
% July 2010 (C)

if nargin==5
    radii2=radii*2;
    colr='blue';
elseif nargin==6
    colr='blue';
end
if size(x,1)==2
    x=x';
    y=y';
    z=z';
end

x(3)=x(2);
x(2)=x(1)+head_frac*(x(3)-x(1));
y(3)=y(2);
y(2)=y(1)+head_frac*(y(3)-y(1));
z(3)=z(2);
z(2)=z(1)+head_frac*(z(3)-z(1));
r=[x(1:2)',y(1:2)',z(1:2)'];

N=50;
dr=diff(r);
dr(end+1,:)=dr(end,:);
origin_shift=(ones(size(r))*(1+max(abs(r(:))))+[dr(:,1) 2*dr(:,2) -dr(:,3)]);
r=r+origin_shift;

normdr=(sqrt((dr(:,1).^2)+(dr(:,2).^2)+(dr(:,3).^2)));
normdr=[normdr,normdr,normdr];
dr=dr./normdr;
Pc=r;
n1=cross(dr,Pc);
normn1=(sqrt((n1(:,1).^2)+(n1(:,2).^2)+(n1(:,3).^2)));
normn1=[normn1,normn1,normn1];
n1=n1./normn1;
P1=n1+Pc;

X1=[];Y1=[];Z1=[];
j=1;
for theta=([0:N])*2*pi./(N);
    R1=Pc+radii*cos(theta).*(P1-Pc) + radii*sin(theta).*cross(dr,(P1-Pc)) -origin_shift;
    X1(2:3,j)=R1(:,1);
    Y1(2:3,j)=R1(:,2);
    Z1(2:3,j)=R1(:,3);
    j=j+1;
end

r=[x(2:3)',y(2:3)',z(2:3)'];

dr=diff(r);
dr(end+1,:)=dr(end,:);
origin_shift=(ones(size(r))*(1+max(abs(r(:))))+[dr(:,1) 2*dr(:,2) -dr(:,3)]);
r=r+origin_shift;

normdr=(sqrt((dr(:,1).^2)+(dr(:,2).^2)+(dr(:,3).^2)));
normdr=[normdr,normdr,normdr];
dr=dr./normdr;
Pc=r;
n1=cross(dr,Pc);
normn1=(sqrt((n1(:,1).^2)+(n1(:,2).^2)+(n1(:,3).^2)));
normn1=[normn1,normn1,normn1];
n1=n1./normn1;
P1=n1+Pc;

j=1;
for theta=([0:N])*2*pi./(N);
    R1=Pc+radii2*cos(theta).*(P1-Pc) + radii2*sin(theta).*cross(dr,(P1-Pc)) -origin_shift;
    X1(4:5,j)=R1(:,1);
    Y1(4:5,j)=R1(:,2);
    Z1(4:5,j)=R1(:,3);
    j=j+1;
end

X1(1,:)=X1(1,:)*0 + x(1);
Y1(1,:)=Y1(1,:)*0 + y(1);
Z1(1,:)=Z1(1,:)*0 + z(1);
X1(5,:)=X1(5,:)*0 + x(3);
Y1(5,:)=Y1(5,:)*0 + y(3);
Z1(5,:)=Z1(5,:)*0 + z(3);

h=surf(X1,Y1,Z1,'facecolor',colr,'edgecolor','none');
end

%% InHull Function

function in = inhull(testpts,xyz,tess,tol)
% inhull: tests if a set of points are inside a convex hull
% usage: in = inhull(testpts,xyz)
% usage: in = inhull(testpts,xyz,tess)
% usage: in = inhull(testpts,xyz,tess,tol)
%
% arguments: (input)
%  testpts - nxp array to test, n data points, in p dimensions
%       If you have many points to test, it is most efficient to
%       call this function once with the entire set.
%
%  xyz - mxp array of vertices of the convex hull, as used by
%       convhulln.
%
%  tess - tessellation (or triangulation) generated by convhulln
%       If tess is left empty or not supplied, then it will be
%       generated.
%
%  tol - (OPTIONAL) tolerance on the tests for inclusion in the
%       convex hull. You can think of tol as the distance a point
%       may possibly lie outside the hull, and still be perceived
%       as on the surface of the hull. Because of numerical slop
%       nothing can ever be done exactly here. I might guess a
%       semi-intelligent value of tol to be
%
%         tol = 1.e-13*mean(abs(xyz(:)))
%
%       In higher dimensions, the numerical issues of floating
%       point arithmetic will probably suggest a larger value
%       of tol.
%
%       DEFAULT: tol = 0
%
% arguments: (output)
%  in  - nx1 logical vector
%        in(i) == 1 --> the i'th point was inside the convex hull.
%  
% Example usage: The first point should be inside, the second out
%
%  xy = randn(20,2);
%  tess = convhulln(xy);
%  testpoints = [ 0 0; 10 10];
%  in = inhull(testpoints,xy,tess)
%
% in = 
%      1
%      0
%
% A non-zero count of the number of degenerate simplexes in the hull
% will generate a warning (in 4 or more dimensions.) This warning
% may be disabled off with the command:
%
%   warning('off','inhull:degeneracy')
%
% See also: convhull, convhulln, delaunay, delaunayn, tsearch, tsearchn
%
% Author: John D'Errico
% e-mail: woodchips@rochester.rr.com
% Release: 3.0
% Release date: 10/26/06
% get array sizes
% m points, p dimensions
p = size(xyz,2);
[n,c] = size(testpts);
if p ~= c
  error 'testpts and xyz must have the same number of columns'
end
if p < 2
  error 'Points must lie in at least a 2-d space.'
end
% was the convex hull supplied?
if (nargin<3) || isempty(tess)
  tess = convhulln(xyz);
end
[nt,c] = size(tess);
if c ~= p
  error 'tess array is incompatible with a dimension p space'
end
% was tol supplied?
if (nargin<4) || isempty(tol)
  tol = 0;
end
% build normal vectors
switch p
  case 2
    % really simple for 2-d
    nrmls = (xyz(tess(:,1),:) - xyz(tess(:,2),:)) * [0 1;-1 0];
    
    % Any degenerate edges?
    del = sqrt(sum(nrmls.^2,2));
    degenflag = (del<(max(del)*10*eps));
    if sum(degenflag)>0
      warning('inhull:degeneracy',[num2str(sum(degenflag)), ...
        ' degenerate edges identified in the convex hull'])
      
      % we need to delete those degenerate normal vectors
      nrmls(degenflag,:) = [];
      nt = size(nrmls,1);
    end
  case 3
    % use vectorized cross product for 3-d
    ab = xyz(tess(:,1),:) - xyz(tess(:,2),:);
    ac = xyz(tess(:,1),:) - xyz(tess(:,3),:);
    nrmls = cross(ab,ac,2);
    degenflag = false(nt,1);
  otherwise
    % slightly more work in higher dimensions, 
    nrmls = zeros(nt,p);
    degenflag = false(nt,1);
    for i = 1:nt
      % just in case of a degeneracy
      % Note that bsxfun COULD be used in this line, but I have chosen to
      % not do so to maintain compatibility. This code is still used by
      % users of older releases.
      %  nullsp = null(bsxfun(@minus,xyz(tess(i,2:end),:),xyz(tess(i,1),:)))';
      nullsp = null(xyz(tess(i,2:end),:) - repmat(xyz(tess(i,1),:),p-1,1))';
      if size(nullsp,1)>1
        degenflag(i) = true;
        nrmls(i,:) = NaN;
      else
        nrmls(i,:) = nullsp;
      end
    end
    if sum(degenflag)>0
      warning('inhull:degeneracy',[num2str(sum(degenflag)), ...
        ' degenerate simplexes identified in the convex hull'])
      
      % we need to delete those degenerate normal vectors
      nrmls(degenflag,:) = [];
      nt = size(nrmls,1);
    end
end
% scale normal vectors to unit length
nrmllen = sqrt(sum(nrmls.^2,2));
% again, bsxfun COULD be employed here...
%  nrmls = bsxfun(@times,nrmls,1./nrmllen);
nrmls = nrmls.*repmat(1./nrmllen,1,p);
% center point in the hull
center = mean(xyz,1);
% any point in the plane of each simplex in the convex hull
a = xyz(tess(~degenflag,1),:);
% ensure the normals are pointing inwards
% this line too could employ bsxfun...
%  dp = sum(bsxfun(@minus,center,a).*nrmls,2);
dp = sum((repmat(center,nt,1) - a).*nrmls,2);
k = dp<0;
nrmls(k,:) = -nrmls(k,:);
% We want to test if:  dot((x - a),N) >= 0
% If so for all faces of the hull, then x is inside
% the hull. Change this to dot(x,N) >= dot(a,N)
aN = sum(nrmls.*a,2);
% test, be careful in case there are many points
in = false(n,1);
% if n is too large, we need to worry about the
% dot product grabbing huge chunks of memory.
memblock = 1e6;
blocks = max(1,floor(n/(memblock/nt)));
aNr = repmat(aN,1,length(1:blocks:n));
for i = 1:blocks
   j = i:blocks:n;
   if size(aNr,2) ~= length(j),
      aNr = repmat(aN,1,length(j));
   end
   in(j) = all((nrmls*testpts(j,:)' - aNr) >= -tol,1)';
end
end
