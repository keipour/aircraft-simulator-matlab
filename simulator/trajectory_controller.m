classdef trajectory_controller < handle

    properties (SetAccess = private, GetAccess = public)
        Waypoints                       % All the waypoints
        NumOfWaypoints                  % The number of waypoints
        TransitionRadius double = 0.25; % When should transition to the next point (in meters)
        CurrentWaypoint int32 = 0;      % The index of the current active waypoint
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
        
        function SetWaypoints(obj, waypoints, radius)
            obj.Waypoints = waypoints;
            obj.NumOfWaypoints = size(waypoints, 1);
            if obj.NumOfWaypoints == 0
                error('At least one waypoint is needed to initialize the trajectory controller.');
            end
            mustBePositive(radius);
            obj.TransitionRadius = radius;
            
            obj.Initialized = true;
            obj.Finished = false;
            obj.CurrentWaypoint = 1;
        end
        
        function next_wp = CalcLookaheadPoint(obj, pose)

            % Switch to the next point if we're within the radius of the
            % current one
            next_wp = obj.Waypoints(obj.CurrentWaypoint, :)';
            if norm(next_wp(1 : 3) - pose(1 : 3)) < obj.TransitionRadius
                if obj.CurrentWaypoint < obj.NumOfWaypoints
                    obj.CurrentWaypoint = obj.CurrentWaypoint + 1;
                    next_wp = obj.Waypoints(obj.CurrentWaypoint, :)';
                else
                    obj.Finished = true;
                end
            end
        end
    end
end

