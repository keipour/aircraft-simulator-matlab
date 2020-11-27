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
            drawnow;
        end
        
        function h = PlotSignalsByName(cols, signals, allinone, gridon, y_label)
            if nargin < 5
                y_label = '';
            end
            if nargin < 4
                gridon = false;
            end
            if nargin < 3
                allinone = false;
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
            
            h = graphics.PlotSignalsFromData(cols, X, T, L, allinone, gridon, y_label);
        end
        
        function h = PlotSignalsFromData(cols, data, times, labels, allinone, gridon, y_label)
            if nargin < 7
                y_label = '';
            end
            if nargin < 6
                gridon = false;
            end
            if nargin < 5
                allinone = false;
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
            legend_labels = {};
            for i = 1 : n_data
                n_datacols = size(data{i}, 2);
                if iscell(data{i})
                    n_datacols = size(data{i}{1}, 2);
                end
                for j = 1 : n_datacols
                    if ~allinone
                        subplot(rows, cols, curr_plot);
                    else
                        hold on
                    end
                    if iscell(data{i})
                        plot_signal(times{i}{1}, data{i}{1}(:, j));
                        if ~allinone
                            plot_signal(times{i}{2}, data{i}{2}(:, j));
                            legend('Measured', 'Commanded');
                        end
                    else
                        plot_signal(times{i}, data{i}(:, j));
                    end
                    if length(labels) >= i || length(labels{i}) >= j
                        y_lbl = cell2mat(labels{i}(:, j));
                        legend_labels = [legend_labels, y_lbl];
                        if n_datacols > 1 && length(labels{i}) > n_datacols
                            lbl = cell2mat(labels{i}(:, n_datacols + 1));
                            if ~allinone
                                lbl = [lbl, ' - ', y_lbl];
                            end
                            title(lbl, 'Interpreter', 'latex');
                        end
                        if allinone
                            ylabel(y_label, 'Interpreter', 'latex');
                        else
                            ylabel(y_lbl, 'Interpreter', 'latex');
                        end
                    end
                    xlabel('Time $[s]$', 'Interpreter', 'latex');
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
            if allinone
                legend(legend_labels);
            end
            drawnow;
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

        function hfig = VisualizeEnvironment(environment)
            hfig = figure;
            support_files.visualize_environment(environment, false);
            view(3);
            drawnow;
        end
        
        function [H_m, H_r] = VisualizeMultirotor(multirotor, axes_only, draw_collision_model)
            if nargin < 2
                axes_only = false;
            end
            if nargin < 3
                draw_collision_model = false;
            end
            figure;
            [H_m, H_r] = support_files.visualize_multirotor(multirotor, axes_only, false, draw_collision_model);
            view(3);
            axis(gca, options.MV_ShowPlotAxes);
            drawnow;
        end
        
        function [H, xyz_limits] = PlotEnvironment(environment)
            [H, limits] = support_files.visualize_environment(environment, true);
            if nargout > 1
                xyz_limits = limits;
            end
        end
        
        function [H_m, H_r] = PlotMultirotor(multirotor)
            [H_m, H_r] = support_files.visualize_multirotor(multirotor, false, true);
        end
        
        function AnimateLoggedTrajectory(multirotor, envronment, zoom_level, ...
                speed, show_info, show_horizon, fpv_cam)
            if nargin < 2
                envronment = environment;
            end
            if nargin < 3
                zoom_level = 0;
            end
            if nargin < 4
                speed = 1;
            end
            if nargin < 5
                show_info = true;
            end
            if nargin < 6
                show_horizon = show_info;
            end
            if nargin < 7
                fpv_cam = [];
            end
                        
            support_files.animate_logged_traj(multirotor, envronment, zoom_level, ...
                speed, show_info, show_horizon, fpv_cam, 0, '');
        end
        
        function RecordLoggedTrajectoryAnimation(filename, fps, multirotor, ...
                envronment, zoom_level, speed, show_info, show_horizon, fpv_cam)
            if nargin < 4
                envronment = environment;
            end
            if nargin < 5
                zoom_level = 0;
            end
            if nargin < 6
                speed = 1;
            end
            if nargin < 7
                show_info = true;
            end
            if nargin < 8
                show_horizon = show_info;
            end
            if nargin < 9
                fpv_cam = [];
            end
            
            support_files.animate_logged_traj(multirotor, envronment, zoom_level, ...
                speed, show_info, show_horizon, fpv_cam, fps, filename);
        end
        
        function PrintDynamicManipulabilityAnalysis(res)
            fprintf('Analysis for the multirotor structure:\n');
            fprintf('    Translation Actuation Type           : %s\n', res.TranslationType);
            fprintf('    Rotation Actuation Type              : %s\n', res.RotationType);
            if isfield(res, 'ActuationRank')
                fprintf('    Robot Actuation Rank                 : %s\n', res.ActuationRank);
            end
            fprintf('    Vertical Thrust Efficiency           : %0.2f %%\n', res.VerticalThrustEfficiency * 100);
            fprintf('    Vertical Translation Efficiency      : %0.2f %%\n', res.VerticalTranslationEfficiency * 100);
            fprintf('    Maximum Vertical Acceleration        : %0.2f m/s^2\n', res.MaximumVerticalAcceleration);
            fprintf('    Maximum Horizontal Acceleration      : %0.2f m/s^2\n', res.MaximumHorizontalAcceleration);
            fprintf('    Maximum Omni-directional Acceleration: %0.2f m/s^2\n', res.AccelerationOmni);
            fprintf('    Bounding Omni Acceleration Vector    : (%0.2f, %0.2f, %0.2f) m/s^2\n', ...
                res.BoundingOmniAcceleration(1), res.BoundingOmniAcceleration(2), res.BoundingOmniAcceleration(3));
            fprintf('    Maximum Angular Acceleration Around X: %0.2f rad/s^2\n', res.MaximumAngularAccelerationX);
            fprintf('    Maximum Angular Acceleration Around Y: %0.2f rad/s^2\n', res.MaximumAngularAccelerationY);
            fprintf('    Maximum Angular Acceleration Around Z: %0.2f rad/s^2\n', res.MaximumAngularAccelerationZ);
        end
        
        function hfig = DrawConvexHull(X, plot_title, label, sphere_radius, sphere_center, ...
                sphere_contact_point, draw_rotation_line, rotation_center, draw_rotation_sphere)
            if nargin < 4
                sphere_radius = 0;
            end
            if nargin < 5
                sphere_center = zeros(1, 3);
            end
            if nargin < 6
                sphere_contact_point = NaN(3, 1);
            end
            if nargin < 7
                draw_rotation_line = false;
            end
            if nargin < 7
                draw_rotation_sphere = false;
            end
            
            facecolor = options.DM_ConvexHullFaceColor;
            linestyle = options.DM_ConvexHullLineStyle;
            edgecolor = options.DM_ConvexHullEdgeColor;
            facealpha = options.DM_ConvexHullFaceAlpha;
            
            hfig = figure;
            r = rank(X, 1e-4);
            if r == 3
                k = convhull(X(:, 1), X(:, 2), X(:, 3), 'Simplify', true);
                trisurf(k, X(:, 1), X(:, 2), X(:, 3), 'FaceColor', facecolor, ...
                    'LineStyle', linestyle, 'EdgeColor', edgecolor, 'FaceAlpha', facealpha);
                
                if sphere_radius > 0
                    [Xs,Ys,Zs] = sphere;
                    Xs = Xs * sphere_radius + sphere_center(1);
                    Ys = Ys * sphere_radius + sphere_center(2);
                    Zs = Zs * sphere_radius + sphere_center(3);
                    hold on
                    surf(Xs, Ys, Zs);
                    if any(~isnan(sphere_contact_point))
                        plot3(sphere_contact_point(1), sphere_contact_point(2), sphere_contact_point(3), 'r*');
                    end
                end
                if draw_rotation_line
                    [Xs,Ys,Zs] = sphere;
                    point_radius = 0.1;
                    Xr = Xs * point_radius + rotation_center(1);
                    Yr = Ys * point_radius + rotation_center(2);
                    Zr = Zs * point_radius + rotation_center(3);
                    Xc = Xs * point_radius + sphere_center(1);
                    Yc = Ys * point_radius + sphere_center(2);
                    Zc = Zs * point_radius + sphere_center(3);
                    hold on
                    surf(Xr, Yr, Zr);
                    surf(Xc, Yc, Zc);
                    plot3([rotation_center(1) sphere_center(1)], [rotation_center(2) sphere_center(2)], ...
                        [rotation_center(3) sphere_center(3)], 'LineWidth', 2, 'Color', options.DM_PointOfRotationToCenterColor);
                end
                if draw_rotation_sphere
                    %figure;
                    draw_tilt_surface(X, sphere_center, rotation_center);
                end

            elseif r == 2
                X_xy = rotate_3d_plane_to_xy(X);
                k = convhull(X_xy(:, 1), X_xy(:, 2), 'Simplify', true);
                fill3(X(k, 1), X(k, 2), X(k, 3), facecolor);
            elseif r == 1
                plot3(X(:, 1), X(:, 2), X(:, 3), 'Color', facecolor);
            end
            xlabel(['$' label '_x$'], 'Interpreter', 'latex');
            ylabel(['$' label '_y$'], 'Interpreter', 'latex');
            zlabel(['$' label '_z$'], 'Interpreter', 'latex');
            axis equal
            title(plot_title);
            set(gca, 'Xdir', 'reverse')
            set(gca, 'Zdir', 'reverse')
            drawnow;
        end
        
        function PlotCrossSections(X, plot_title, label, z_from_zero, ...
                plot_crosssection_x, plot_crosssection_y, plot_crosssection_z, ...
                sphere_radius, sphere_center)
            if rank(X) < 3
                return;
            end
            if nargin < 4
                z_from_zero = false;
            end
            if nargin < 5
               plot_crosssection_x = true;
               plot_crosssection_y = true;
               plot_crosssection_z = true;
            end
            if nargin < 6
                sphere_radius = 0;
            end
            if nargin < 7
                sphere_center = zeros(1, 3);
            end
            
            sprows = options.DM_CrossSectionSubplotRows;
            spcols = options.DM_CrossSectionSubplotCols;
            if plot_crosssection_x
                plot_cross_section(X, plot_title, label, 'x', sprows, spcols, z_from_zero, ...
                    plot_crosssection_x, plot_crosssection_y, plot_crosssection_z);
            end
            if plot_crosssection_y
                plot_cross_section(X, plot_title, label, 'y', sprows, spcols, z_from_zero, ...
                    plot_crosssection_x, plot_crosssection_y, plot_crosssection_z);
            end
            if plot_crosssection_z
                plot_cross_section(X, plot_title, label, 'z', sprows, spcols, z_from_zero, ...
                    plot_crosssection_x, plot_crosssection_y, plot_crosssection_z);
            end
            plot_center_cross_sections(X, plot_title, label, ...
                plot_crosssection_x, plot_crosssection_y, plot_crosssection_z, ...
                sphere_radius, sphere_center);
            drawnow;
        end
        
        function PlotLateralThrustDynInv(mult, X, des_ang_accel, plot_title, label)
            if rank(X) < 3
                return;
            end
            
            sprows = options.DM_CrossSectionSubplotRows;
            spcols = options.DM_CrossSectionSubplotCols;
            nsubplots = sprows * spcols;
            
            ca = control_allocation(mult);
            figure;
            sgtitle(plot_title);
            Zq = linspace(min(X(:, 3)), 0, nsubplots);
            X_xy = X(:, 1:2);
            xylimits = [min(X_xy(:)), max(X_xy(:))];
            for i = 1 : nsubplots
                subplot(sprows, spcols, i);
                is_angular = false;
                [xc, yc] = get_cross_section_from_dynamic_inversion ...
                    (X(:, 1), X(:, 2), Zq(i), ca, mult, is_angular, des_ang_accel);
                if ~isempty(xc)
                    k = convhull(xc, yc, 'Simplify', true);
                    fill(xc(k), yc(k), options.DM_LateralThrustColor);
                end
                xlabel(['$' label '_x$'], 'Interpreter', 'latex');
                ylabel(['$' label '_y$'], 'Interpreter', 'latex');
                title(['$' label '_z = ' num2str(Zq(i), '%0.2f') '$'], 'Interpreter', 'latex');
                xlim(xylimits);
                ylim(xylimits);
            end
            drawnow;
        end
    end
end

%% Helper functions
function draw_tilt_surface(X, sphere_center, rotation_center)
    
    radius = norm(sphere_center - rotation_center);
    
    Nq = options.DM_CrossSectionPoints;
    minx = min(X(:, 1));
    miny = min(X(:, 2));
    maxx = max(X(:, 1));
    maxy = max(X(:, 2));
    
    Xq = rand(Nq, 1) * (maxx - minx) + minx;
    Yq = rand(Nq, 1) * (maxy - miny) + miny;
    dx = Xq - rotation_center(1);
    dy = Yq - rotation_center(2);
    Zq = rotation_center(3) - sqrt(radius.^2 - dx.*dx - dy.*dy);
    
    in = math.InConvexHull([Xq, Yq, Zq], X);
    
    x = Xq(in);
    y = Yq(in);
    z = Zq(in);
    dt = delaunayTriangulation(x, y);
    tri = dt.ConnectivityList;
    xi = dt.Points(:, 1); 
    yi = dt.Points(:, 2);
    F = scatteredInterpolant(x, y, z);
    zi = F(xi, yi) ;
    
    hold on
    ts = trisurf(tri,xi,yi,zi);
    set(ts, 'EdgeColor', 'interp', 'FaceColor', 'interp');
    view(3)
%     camlight
%     lighting flat
end

function hfig = plot_cross_section(X, plot_title, label, pivot_axis, csrows, cscols, z_from_zero, ...
    plot_crosssection_x, plot_crosssection_y, plot_crosssection_z)

    nsubplots = csrows * cscols;
    if lower(pivot_axis) == 'z'
        if ~plot_crosssection_z
            return;
        end
        axis_labels = {'x', 'y', 'z'};
        query_points = linspace(min(X(:, 3)), max(X(:, 3)), nsubplots);
        if z_from_zero && max(X(:, 3)) > 0
            query_points = linspace(min(X(:, 3)), 0, nsubplots);
        end
        axis1 = X(:, 1);
        axis2 = X(:, 2);
        axis3 = X(:, 3);
    elseif lower(pivot_axis) == 'y'
        if ~plot_crosssection_y
            return;
        end
        axis_labels = {'x', 'z', 'y'};
        query_points = linspace(min(X(:, 2)), max(X(:, 2)), nsubplots);
        axis1 = X(:, 1);
        axis2 = X(:, 3);
        axis3 = X(:, 2);
    elseif lower(pivot_axis) == 'x'
        if ~plot_crosssection_x
            return;
        end
        axis_labels = {'y', 'z', 'x'};
        query_points = linspace(min(X(:, 1)), max(X(:, 1)), nsubplots);
        axis1 = X(:, 2);
        axis2 = X(:, 3);
        axis3 = X(:, 1);
    end
    
    hfig = figure;
    sgtitle([plot_title ' (' upper(axis_labels{3}) ' axis)']);

   try
        for i = 1 : nsubplots
            subplot(csrows, cscols, i);
            plot_single_cross_section(axis1, axis2, axis3, query_points(i), label, axis_labels);
            if lower(pivot_axis) ~= 'z'
                set(gca, 'Ydir', 'reverse');
            end

        end
    catch
        close(hfig);
    end
end

function hfig = plot_center_cross_sections(X, plot_title, label, ...
    plot_crosssection_x, plot_crosssection_y, plot_crosssection_z, ...
    sphere_radius, sphere_center)

    nsubplots = double(plot_crosssection_x) + double(plot_crosssection_y) + double(plot_crosssection_z);
    if nsubplots == 0 || sphere_radius == 0
        return;
    end
    spcounter = 0;
    
    hfig = figure;
    if nsubplots == 1
        title(plot_title);
    else
        sgtitle(plot_title);
    end

    if plot_crosssection_x
        if nsubplots > 1
            spcounter = spcounter + 1;
            subplot(1, nsubplots, spcounter);
        end
        plot_single_cross_section(X(:, 2), X(:, 3), X(:, 1), sphere_center(1), label, {'y', 'z', 'x'}, sphere_radius, sphere_center([2, 3]));
        set(gca, 'Ydir', 'reverse');
    end
    if plot_crosssection_y
        if nsubplots > 1
            spcounter = spcounter + 1;
            subplot(1, nsubplots, spcounter);
        end
        plot_single_cross_section(X(:, 1), X(:, 3), X(:, 2), sphere_center(2), label, {'x', 'z', 'y'}, sphere_radius, sphere_center([1, 3]));
        set(gca, 'Ydir', 'reverse');
    end
    if plot_crosssection_z
        if nsubplots > 1
            spcounter = spcounter + 1;
            subplot(1, nsubplots, spcounter);
        end
        plot_single_cross_section(X(:, 1), X(:, 2), X(:, 3), sphere_center(3), label, {'x', 'y', 'z'}, sphere_radius, sphere_center([1, 2]));
    end
end

function plot_single_cross_section(axis1, axis2, axis3, axis3_query, label, axis_labels, sphere_radius, sphere_center)

    if nargin < 7
        sphere_radius = 0;
    end
    if nargin < 8
        sphere_center = zeros(1, 3);
    end

    xlimit = max(abs(axis1));
    ylimit = max(abs(axis2));
    xlimit = max(sphere_radius + sphere_center(1), xlimit);
    ylimit = max(sphere_radius + sphere_center(2), ylimit);
    
    [xc, yc] = get_cross_section(axis1, axis2, axis3, axis3_query);
    if ~isempty(xc)
        k = convhull(xc, yc, 'Simplify', true);
        fill(xc(k), yc(k), options.DM_CrossSectionColor);
        circle(sphere_center(1), sphere_center(2), sphere_radius, options.DM_AccelerationOmniSphereColor);
    end
    xlabel(['$' label '_' axis_labels{1} '$'], 'Interpreter', 'latex');
    ylabel(['$' label '_' axis_labels{2} '$'], 'Interpreter', 'latex');
    title(['$' label '_' axis_labels{3} ' = ' num2str(axis3_query, '%0.2f') '$'], 'Interpreter', 'latex');
    xlim([-xlimit xlimit]);
    ylim([-ylimit ylimit]);
    daspect([1 1 1]);
end

function [x, y] = get_cross_section(X, Y, Z, z)
    Nq = options.DM_CrossSectionPoints;
    minx = min(X);
    miny = min(Y);
    maxx = max(X);
    maxy = max(Y);
    
    Xq = rand(Nq, 1) * (maxx - minx) + minx;
    Yq = rand(Nq, 1) * (maxy - miny) + miny;
    
    in1 = math.InConvexHull([Xq, Yq, z*ones(Nq, 1)], [X, Y, Z]);
    x = Xq(in1);
    y = Yq(in1);
end

function [x, y] = get_cross_section_from_dynamic_inversion(X, Y, z, ca, m, is_angluar, des_accel)
    if nargin < 6
        is_angluar = false;
    end
    Nq = options.DM_LateralThrustMonteCarloPoints;
    minx = min(X);
    miny = min(Y);
    maxx = max(X);
    maxy = max(Y);
    
    Xq = rand(Nq, 1) * (maxx - minx) + minx;
    Yq = rand(Nq, 1) * (maxy - miny) + miny;
    
    out1 = false(Nq, 1);
    for i = 1 : Nq
        if is_angluar
            [~, out1(i)] = ca.CalcRotorSpeeds(m, des_accel, [Xq(i); Yq(i); z]);
        else
            [~, out1(i)] = ca.CalcRotorSpeeds(m, [Xq(i); Yq(i); z], des_accel);
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

    skip = 1; %397; % reduce tne number of points for matlab2tikz conversion
    
    if length(t) < skip * 10
        skip = 1;
    end
    
    % Plot the signal
    if nargin < 3
        plot(t(1:skip:end), Y(1:skip:end), 'LineWidth', 2);
    elseif nargin < 4
        plot(t(1:end), Y(1:end), properties);
    else
        plot(t(1:end), Y(1:end), properties, 'LineWidth', line_width);
    end
    
    hold off
end
        
function fix_plot_limits(Xs, Ys)
    
    axh = findobj( gcf, 'Type', 'Axes' );
    axh_children = get(axh(1), 'Children');
    is_first_plot = length(axh_children) < 2;
    
    max_x = -inf;
    min_x = inf;
    max_y = -inf;
    min_y = inf;

    y_padding = 0.5;
    
    if ~is_first_plot
        curr_y_lim = ylim;
        curr_x_lim = xlim;
        max_x = curr_x_lim(2);
        min_x = curr_x_lim(1);
        max_y = curr_y_lim(2) - y_padding;
        min_y = curr_y_lim(1) + y_padding;
    end
    
    for i = 1 : length(Xs)
        max_x = max(max_x, max(Xs{i}));
        min_x = min(min_x, min(Xs{i}));
    end
    for i = 1 : length(Ys)
        max_y = max(max_y, max(Ys{i}));
        min_y = min(min_y, min(Ys{i}));
    end
    xlim([min_x max_x]);
    ylim([min_y - y_padding, max_y + y_padding]);
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

function h = circle(centerx, centery, radius, color)
    if radius <= 0
        return;
    end
    N = 100;
    hold on
    th = linspace(0, 2 * pi, N);
    rho = ones(1, N) * radius;
    [x, y] = pol2cart(th, rho);
    x = x + centerx;
    y = y + centery;
    h = fill(x, y, color);
    plot(centerx, centery, 'k.', 'LineWidth', 2);
    hold off
end
