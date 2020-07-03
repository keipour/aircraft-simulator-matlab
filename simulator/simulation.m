%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% Copyright (c) 2020 Carnegie Mellon University
% This tool has been developed for educational purposes only as a 
% control tutorial in Air Lab Summer School 2020 (https://theairlab.org). 
% For License information please see the LICENSE file in the root directory.
% Author: Azarakhsh Keipour (keipour [at] cmu.edu)
% Please contact us for any questions or issues.
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

classdef simulation < handle
    properties
        TotalTime = 5;      % in secs
        TimeStep = 1e-3;    % in secs
        Multirotor multirotor
        Controller controller
    end
    
    properties(SetAccess=protected, GetAccess=public)
        CurrentTime = 0;    % in secs
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
        
        function set.TotalTime(obj, value)
            obj.TotalTime = value;      % in secs
            obj.Reset();
        end

        function set.TimeStep(obj, value)
            obj.TimeStep = value;      % in secs
            obj.Reset();
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
        
        function NextStepPlant(obj, rotor_speeds_squared)
        % Update the plant state for the next time step and advance time
        
            obj.Multirotor.UpdateState(rotor_speeds_squared, obj.TimeStep);
            obj.StateHistory.PushBack(obj.Multirotor.State);
            obj.CurrentTime = obj.CurrentTime + obj.TimeStep;
        end
        
        function rotor_speeds_squared = NextAttitudeCommand(obj, rpy_des, lin_accel)
        % Calculate the multirotor command for a desired attitude
        
            rotor_speeds_squared = obj.Controller.ControlAttitude(obj.Multirotor, rpy_des, lin_accel, obj.TimeStep);
        end
        
        function rotor_speeds_squared = NextPositionCommand(obj, pos_des, yaw_des)
        % Calculate the multirotor command for a desired position and yaw

            rotor_speeds_squared = obj.Controller.ControlPosition(obj.Multirotor, pos_des, yaw_des, obj.TimeStep);
        end
        
        function res = SimulateAttitudeResponse(obj, rpy_des, plot)
        % Simulate the response to a desired attitude input
            
            obj.Reset();
            lin_accel = zeros(3, 1);
            while true
                u = obj.NextAttitudeCommand(rpy_des, lin_accel);
                obj.NextStepPlant(u);
                if obj.IsLastStep()
                    break;
                end
            end
            
            % Analysis of the response
            signal_names = {'Roll', 'Pitch', 'Yaw'};
            res = analysis.AnalyzeAndOutputResponse(obj.GetTimeSteps(), ...
                obj.StateHistory.GetRPYs(), rpy_des, signal_names, plot);
        end
        
        function res = SimulatePositionResponse(obj, pos_des, yaw_des, plot)
        % Simulate the response to a desired attitude input
            
            obj.Reset();
            while true
                u = obj.NextPositionCommand(pos_des, yaw_des);
                obj.NextStepPlant(u);
                if obj.IsLastStep()
                    break;
                end
            end
            
            % Analysis of the response
            signal_names = {'X', 'Y', 'Z', 'Yaw'};
            pos_res = obj.StateHistory.GetPositions();
            rpy_res = obj.StateHistory.GetRPYs();
            res = analysis.AnalyzeAndOutputResponse(obj.GetTimeSteps(), ...
                [pos_res, rpy_res(:, 3)], [pos_des; yaw_des], signal_names, plot);
        end
        
        function GetAccelerationDistribution(obj)
            flag = true;
            Fx = [];
            Fy = [];
            Fz = [];
            rotor_speeds_squared = zeros(obj.Multirotor.NumOfRotors, 1);
            mins = zeros(obj.Multirotor.NumOfRotors, 1);
            maxs = zeros(obj.Multirotor.NumOfRotors, 1);
            for i = 1 : obj.Multirotor.NumOfRotors
                maxs(i) = obj.Multirotor.Rotors{i}.MaxrotorSpeedSquared;
            end
            while flag
                obj.Multirotor.UpdateState(rotor_speeds_squared, 0);
                Fx = [Fx; obj.Multirotor.State.Force(1)];
                Fy = [Fy; obj.Multirotor.State.Force(2)];
                Fz = [Fz; obj.Multirotor.State.Force(3)];
                [rotor_speeds_squared, flag] = NextPerm(rotor_speeds_squared, mins, maxs, 4);
            end
            ax = Fx / obj.Multirotor.Mass;
            ay = Fy / obj.Multirotor.Mass;
            az = Fz / obj.Multirotor.Mass;
            k = convhull(ax, ay, az);
            figure;
            trisurf(k, ax, ay, az, 'FaceColor','cyan', 'LineStyle', '-');
            xlabel('a_x');
            ylabel('a_y');
            zlabel('a_z');
            axis equal

            figure;
            Zq = linspace(min(az), 0, 9);
            xylimits = [min(min(ax), min(ay)), max(max(ax), max(ay))];
            for i = 1 : 9
                subplot(3, 3, i);
                [xc, yc] = CrossSection(ax, ay, az, Zq(i));
                plot(xc, yc);
                xlabel('a_x');
                ylabel('a_y');
                title(['a_z = ', num2str(Zq(i))]);
                xlim(xylimits);
                ylim(xylimits);
                axis equal
            end
            %figure; 
            %plot(ax, ay, 'o')
        end
    end
end

%% Helper functions
function [perm, flag] = NextPerm(vec, mins, maxs, n_steps)
    steps = (maxs - mins) / n_steps;
    n = length(vec);
    perm = vec;
    curr = n;
    while curr > 0
        if perm(curr) < maxs(curr) - 1e-3
            perm(curr) = perm(curr) + steps(curr);
            flag = true;
            return;
        end
        for i = curr : n
            perm(i) = mins(i);
        end
        curr = curr - 1;
    end
    flag = false;
end

function [x, y] = CrossSection(X, Y, Z, z)
    Nq = 1e6;
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