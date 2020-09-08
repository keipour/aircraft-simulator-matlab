classdef queue < handle
    properties(SetAccess = protected, GetAccess = public)
        Data = {};
        Times = {};
    end
    
    properties(SetAccess = protected, GetAccess = protected)
        Lengths = [];
    end
    
    methods
        function obj = queue(rows)
            rows = uint32(rows);
            obj.Reset(rows);
        end
        
        function Reset(obj, rows)
            if nargin < 2
                rows = 0;
            end
            rows = uint32(rows);
            obj.Data = cell(rows, 1);
            obj.Times = cell(rows, 1);
            obj.Lengths = zeros(rows, 1);
            for i = 1 : rows
                obj.Data{i} = {};
                obj.Times{i} = [];
            end
        end
        
        function Add(obj, row, data, time)
            len = obj.Lengths(row, 1) + 1;
            obj.Lengths(row, 1) = len;
            obj.Data{row}{len} = data;
            obj.Times{row}(len) = time;
        end
        
        function [data, times] = Get(obj, row)
            row = uint32(row);
            data = obj.Data{row}';
            times = obj.Times{row}';
        end
    end
end
