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

        function AnalyzeAccelerationDynamicManipulability(multirotor, n_steps)
            n_rotors = multirotor.NumOfRotors;
            n_total = n_steps ^ n_rotors;
            accel = zeros(n_total, 3);
            mins = zeros(n_rotors, 1);
            maxs = zeros(n_rotors, 1);
            for i = 1 : n_rotors
                maxs(i) = multirotor.Rotors{i}.MaxrotorSpeedSquared;
            end
            
            steps = (maxs - mins) ./ (n_steps - 1);

            for i = 1 : n_total
                nextnum = dec2base(i - 1, n_steps, n_rotors) - '0';
                rotor_speeds_squared = nextnum' .* steps;
                accel(i, :) = multirotor.CalculateAccelerationManipulability(rotor_speeds_squared);
            end
            
            graphics.DrawConvexHull(accel, 'Dynamic Manipulability - Acceleration', 'a');

%             tic
%             figure;
%             Zq = linspace(min(accel(:, 3)), 0, 9);
%             accel_xy = accel(:, 1:2);
%             xylimits = [min(accel_xy(:)), max(accel_xy(:))];
%             for i = 1 : 9
%                 subplot(3, 3, i);
%                 [xc, yc] = CrossSection(accel(:, 1), accel(:, 2), accel(:, 3), Zq(i));
%                 plot(xc, yc);
%                 xlabel('a_x');
%                 ylabel('a_y');
%                 title(['a_z = ', num2str(Zq(i))]);
%                 xlim(xylimits);
%                 ylim(xylimits);
%                 axis equal
%             end
%             toc
            
%            tic
%            ca = control_allocation(multirotor);
%            figure;
%            Zq = linspace(min(accel(:, 3)), 0, 9);
%            accel_xy = accel(:, 1:2);
%            xylimits = [min(accel_xy(:)), max(accel_xy(:))];
%            for i = 1 : 9
%                subplot(3, 3, i);
%                [xc, yc] = CrossSection2(accel(:, 1), accel(:, 2), Zq(i), ca, multirotor);
%                plot(xc, yc);
%                xlabel('a_x');
%                ylabel('a_y');
%                title(['a_z = ', num2str(Zq(i))]);
%                xlim(xylimits);
%                ylim(xylimits);
%                axis equal
%            end
%            toc
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

function [x, y] = CrossSection(X, Y, Z, z)
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

function [x, y] = CrossSection2(X, Y, z, ca, m)
    Nq = 1e4;
    minx = min(X);
    miny = min(Y);
    maxx = max(X);
    maxy = max(Y);
    
    Xq = rand(Nq, 1) * (maxx - minx) + minx;
    Yq = rand(Nq, 1) * (maxy - miny) + miny;
    
    out1 = false(Nq, 1);
    for i = 1 : Nq
        [~, out1(i)] = ca.CalcRotorSpeeds(m, [Xq(i); Yq(i); z], zeros(3, 1));
    end
    x = Xq(~out1);
    y = Yq(~out1);
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
