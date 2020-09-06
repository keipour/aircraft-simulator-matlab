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
        
        function h = PlotSignalsByName(cols, signals, gridon)
            if nargin < 3
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

        function VisualizeEnvironment(environment)
            figure;
            support_files.visualize_environment(environment, false);
            view(3);
            drawnow;
        end
        
        function H = VisualizeMultirotor(multirotor, axes_only, draw_collision_model)
            if nargin < 2
                axes_only = false;
            end
            if nargin < 3
                draw_collision_model = false;
            end
            figure;
            H = support_files.visualize_multirotor(multirotor, axes_only, false, draw_collision_model);
            view(3);
            drawnow;
        end
        
        function PlotEnvironment(environment)
            support_files.visualize_environment(environment, true);
        end
        
        function H = PlotMultirotor(multirotor)
            H = support_files.visualize_multirotor(multirotor, false, true);
        end
        
        function AnimateLoggedTrajectory(multirotor, envronment, zoom_level, ...
                speed, show_info, show_horizon, show_fpv)
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
                show_fpv = show_info;
            end
                        
            support_files.animate_logged_traj(multirotor, envronment, zoom_level, ...
                speed, show_info, show_horizon, show_fpv, 0, '');
        end
        
        function RecordLoggedTrajectoryAnimation(filename, fps, multirotor, ...
                envronment, zoom_level, speed, show_info, show_horizon, show_fpv)
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
                show_fpv = show_info;
            end
            
            support_files.animate_logged_traj(multirotor, envronment, zoom_level, ...
                speed, show_info, show_horizon, show_fpv, fps, filename);
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
            drawnow;
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
            drawnow;
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
