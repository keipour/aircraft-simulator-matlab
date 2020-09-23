classdef trajectory_controller < handle

    properties (SetAccess = private, GetAccess = public)
        Waypoints(:, 1) support_files.waypoint  % All the waypoints
        NumOfWaypoints                          % The number of waypoints
        PositionThreshold double                % When should transition to the next point (in meters)
        RPYThreshold double                     % When should transition to the next point (in degrees)
        ForceThreshold double                   % When should transition to the next point (in Newtons)
        CurrentWaypoint int32 = 0;              % The index of the current active waypoint
    end
    
    properties (SetAccess = private, GetAccess = private)
        Initialized = false;            % Is the trajectory initialized?
        Finished = false;               % Is the trajectory finished?
    end
    
    methods
        
        function flag = IsInitialized(obj)
            flag = obj.Initialized;
        end
        
        function flag = IsFinished(obj)
            flag = obj.Finished;
        end
        
        function SetWaypoints(obj, waypoints, pos_thresh, rpy_thresh, force_thresh)
            obj.Waypoints = support_files.waypoint.ProcessWaypoints(waypoints);
            obj.NumOfWaypoints = length(obj.Waypoints);
            if obj.NumOfWaypoints == 0
                error('At least one waypoint is needed to initialize the trajectory controller.');
            end
            obj.PositionThreshold = pos_thresh;
            obj.RPYThreshold = rpy_thresh;
            obj.ForceThreshold = force_thresh;
            
            obj.Initialized = true;
            obj.Finished = false;
            obj.CurrentWaypoint = 1;
        end
        
        function next_wp = CalcLookaheadPoint(obj, pos, rpy, force, in_contact)

            if length(rpy) == 1
                rpy = [0; 0; rpy];
            end
            if nargin < 4
                force = [];
            end
            if nargin < 5
                in_contact = false;
            end
            if isempty(force)
                force = 0;
            end
            
            % Switch to the next point if we're within the radius of the
            % current one. Not issuing lookahead for now.
            next_wp = obj.Waypoints(obj.CurrentWaypoint);
            if obj.HasReached(next_wp, pos, rpy, force, in_contact)
                if obj.CurrentWaypoint < obj.NumOfWaypoints
                    obj.CurrentWaypoint = obj.CurrentWaypoint + 1;
                    next_wp = obj.Waypoints(obj.CurrentWaypoint);
                else
                    obj.Finished = true;
                end
            end
        end
    end
    
    methods (Access = private)
        function flag = HasReached(obj, next_wp, pos, rpy, force, in_contact)

            persistent last_contact_status
            if isempty(last_contact_status)
                last_contact_status = false;
            end
            
            flag = true;
            
            if last_contact_status == false && in_contact && ~next_wp.HasForce()
                return;
            end

            last_contact_status = in_contact;
            
            if length(obj.PositionThreshold) == 1 % it's a radius on position
                if norm(next_wp.Position - pos) > obj.PositionThreshold
                    flag = false;
                    return;
                end
            elseif length(obj.PositionThreshold) == 2 % it's a radius on xy and threshold on z
                if norm(next_wp.Position(1 : 2) - pos(1 : 2)) > obj.PositionThreshold(1) ...
                        || abs(next_wp.Position(3) - pos(3)) > obj.PositionThreshold(2)
                    flag = false;
                    return;
                end
            elseif length(obj.PositionThreshold) == 3 % it's a threshold on x, y and z
                if abs(next_wp.Position(1) - pos(1)) > obj.PositionThreshold(1) ...
                        || abs(next_wp.Position(2) - pos(2)) > obj.PositionThreshold(2) ...
                        || abs(next_wp.Position(3) - pos(3)) > obj.PositionThreshold(3)
                    flag = false;
                    return;
                end
            end
            if abs(next_wp.RPY(1) - rpy(1)) > obj.RPYThreshold(1) ...
                    || abs(next_wp.RPY(2) - rpy(2)) > obj.RPYThreshold(2) ...
                    || abs(next_wp.RPY(3) - rpy(3)) > obj.RPYThreshold(3)
                flag = false;
                return;
            end
            if length(obj.ForceThreshold) == 1 % it's a threshold on normal force
                if norm(next_wp.Force(1) - force(1)) > obj.PositionThreshold
                    flag = false;
                    return;
                end
            elseif length(obj.ForceThreshold) == 3 % it's a threshold on Fx, Fy and Fz
                if abs(next_wp.Force(1) - force(1)) > obj.ForceThreshold(1) ...
                        || abs(next_wp.Force(2) - force(2)) > obj.ForceThreshold(2) ...
                        || abs(next_wp.Force(3) - force(3)) > obj.ForceThreshold(3)
                    flag = false;
                    return;
                end
            end
        end
    end
end
