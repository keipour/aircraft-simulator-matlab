classdef data_field < handle
    properties
        Data = [];
        Time = 0;
    end
    
    methods
        function Reset(obj)
            obj.Data = [];
            obj.Time = 0;
        end
        
        function flag = IsInitialized(obj)
            flag = ~isempty(obj.Data);
        end
        
        function Set(obj, data, time)
            obj.Data = data;
            obj.Time = time;
        end
    end
end
