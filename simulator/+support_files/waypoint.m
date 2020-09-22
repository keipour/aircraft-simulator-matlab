classdef waypoint < handle
    
    properties
        Position
        RPY
        Force
    end
    
    methods
        function obj = waypoint(input, rpy, force)
            if nargin == 1
                wp = support_files.waypoint.ProcessWaypoints(input);
                if length(wp) > 1
                    error('Only one waypoint can be initialized.');
                end
            else
                obj.Position = input;
                obj.RPY = rpy;
                obj.Force = force;
            end
        end
        
        function flag = HasRPY(obj)
            flag = any(isnan(obj.RPY));
        end
        
        function flag = HasForce(obj)
            flag = any(obj.Force);
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
                            w = support_files.waypoint(input{i}(j, 1 : 3)', [NaN; NaN; input{i}(j, 4)], zeros(3, 1));
                        case 6
                            w = support_files.waypoint(input{i}(j, 1 : 3)', input{i}(j, 4 : 6)', zeros(3, 1));
                        case 7
                            w = support_files.waypoint(input{i}(j, 1 : 3)', [NaN; NaN; input{i}(j, 4)], input{i}(j, 5 : 7)');
                        case 9
                            w = support_files.waypoint(input{i}(j, 1 : 3)', input{i}(j, 4 : 6)', input{i}(j, 7 : 9)');
                        otherwise
                            error('Error processing the input waypoints.');
                    end
                    waypoints = [waypoints; w];
                end
            end
        end
    end
end

