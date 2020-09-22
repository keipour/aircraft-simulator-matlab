classdef waypoint < handle
    
    properties
        Type
        Position
        RPY
        Force
    end
    
    properties (Constant)
        Types_PositionYaw = 0;
        Types_PositionRPY = 1;
        Types_PositionYawForce = 2;
        Types_PositionRPYForce = 3;
    end
    
    methods
        function obj = waypoint(input, pos, rpy, force)
            if nargin == 1
                obj = support_files.waypoint.ProcessWaypoints(input);
            else
                obj.Type = input;
                obj.Position = pos;
                obj.RPY = rpy;
                obj.Force = force;
            end
        end
        
    end
    
    methods (Static)
        function waypoints = ProcessWaypoints(input)
        % Input can be a matrix or a cell array. A matrix is interpreted
        % like a single-cell array. Each cell is one of these options:
        % N x 4 matrix: Position, yaw
        % N x 6 matrix: Position, roll, pitch, yaw
        % N x 7 matrix: Position, yaw, force
        % N x 9 matrix: Position, roll, pitch, yaw, force

            if ~iscell(input)
                input = {input};
            end
            
            waypoints = [];
            for i = 1 : length(input)
                for j = 1 : size(input{i}, 1)
                    w = [];
                    switch size(input{i}, 2)
                        case 4
                            w = support_files.waypoint(support_files.waypoint.Types_PositionYaw, ...
                                input{i}(j, 1 : 3)', [NaN; NaN; input{i}(j, 4)], NaN(3, 1));
                        case 6
                            w = support_files.waypoint(support_files.waypoint.Types_PositionRPY, ...
                                input{i}(j, 1 : 3)', input{i}(j, 4 : 6)', NaN(3, 1));
                        case 7
                            w = support_files.waypoint(support_files.waypoint.Types_PositionYawForce, ...
                                input{i}(j, 1 : 3)', [NaN; NaN; input{i}(j, 4)], input{i}(j, 5 : 7)');
                        case 9
                            w = support_files.waypoint(support_files.waypoint.Types_PositionRPYForce, ...
                                input{i}(j, 1 : 3)', input{i}(j, 4 : 6)', input{i}(j, 7 : 9)');
                        otherwise
                            error('Error processing the input waypoints.');
                    end
                    waypoints = [waypoints; w];
                end
            end
        end
    end
end

