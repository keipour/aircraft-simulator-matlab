classdef trajectory_controller < handle

    properties
        WaypointFollower
        LookAheadDistance double
    end
    
    properties (SetAccess = private, GetAccess = private)
        Initialized = false;
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
            obj.Initialized = true;
        end
        
        function [lookahead_pose, finished] = CalcLookaheadPoint(obj, pose)
            pose = [pose(1 : 3); deg2rad(pose(4))];
            [lookahead_point, ~, desired_yaw, ~, finished] = obj.WaypointFollower(pose, obj.LookAheadDistance);
            lookahead_pose = [lookahead_point; rad2deg(desired_yaw)];
        end
    end
end

