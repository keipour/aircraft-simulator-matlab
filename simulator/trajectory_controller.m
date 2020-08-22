classdef trajectory_controller < handle

    properties
        WaypointFollower
        LookAheadDistance double
    end
    
    properties (SetAccess = private, GetAccess = private)
        Initialized = false;
        Finished = false;
        Waypoints
        LastWaypoint
    end
    
    methods
        
        function flag = IsInitialized(obj)
            flag = obj.Initialized;
        end
        
        function SetWaypoints(obj, waypoints, radius, lookahead_dist)
            poses = waypoints(:, 1:3);
            yaws = deg2rad(waypoints(:, 4));
            obj.WaypointFollower = uavWaypointFollower(...
                'UAVType', 'multirotor', 'Waypoints', poses, ...
                'YawAngles', yaws, 'TransitionRadius', radius);
            obj.LookAheadDistance = lookahead_dist;
            obj.Waypoints = waypoints;
            obj.LastWaypoint = waypoints(size(waypoints, 1), :)';
            obj.Initialized = true;
        end
        
        function [lookahead_pose, finished] = CalcLookaheadPoint(obj, pose)

            % Return the last waypoint if the sequense is finished
            if obj.Finished
                lookahead_pose = obj.LastWaypoint;
                finished = true;
                return;
            end
                
            % Calculate the lookahead point
            pose = [pose(1 : 3); deg2rad(pose(4))];

            [lookahead_point, ~, desired_yaw, ~, finished] = obj.WaypointFollower(pose, obj.LookAheadDistance);

            % Check if the sequense is finished and we need to send the
            % last waypoint instead of the lookahead
            if finished > 0
                obj.Finished = true;
                lookahead_pose = obj.LastWaypoint;
            else
                % Otherwise send the lookahead
                lookahead_pose = [lookahead_point; rad2deg(desired_yaw)];
            end
            
        end
    end
end

