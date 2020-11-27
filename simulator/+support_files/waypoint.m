classdef waypoint < handle
    
    properties
        Position
        RPY
        Force
        ServoAngles
    end
    
    methods
        function obj = waypoint(input, rpy, force, servo_angles)
            if nargin == 1
                obj = support_files.waypoint.ProcessWaypoints(input);
                if length(obj) > 1
                    error('Only one waypoint can be initialized.');
                end
            else
                obj.Position = input;
                obj.RPY = rpy;
                obj.Force = force;
            end
            
            if nargin < 4
                obj.ServoAngles = NaN;
            else
                obj.ServoAngles = servo_angles;
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
                if size(input, 2) == 1
                    input = input';
                end
                input = {input};
            end
            
            waypoints = [];
            for i = 1 : size(input, 1)
                desinput = input{i, 1};
                servoinp = NaN;
                if size(input, 2) == 2
                    servoinp = input{i, 2};
                end
                
                for j = 1 : size(desinput, 1)
                    w = [];
                    switch size(desinput, 2)
                        case 4
                            w = support_files.waypoint(desinput(j, 1 : 3)', [NaN; NaN; desinput(j, 4)], zeros(3, 1), servoinp);
                        case 6
                            w = support_files.waypoint(desinput(j, 1 : 3)', desinput(j, 4 : 6)', zeros(3, 1), servoinp);
                        case 7
                            w = support_files.waypoint(desinput(j, 1 : 3)', [NaN; NaN; desinput(j, 4)], desinput(j, 5 : 7)', servoinp);
                        case 9
                            w = support_files.waypoint(desinput(j, 1 : 3)', desinput(j, 4 : 6)', desinput(j, 7 : 9)', servoinp);
                        otherwise
                            error('Error processing the input waypoints.');
                    end
                    waypoints = [waypoints; w];
                end
            end
        end
    end
end

