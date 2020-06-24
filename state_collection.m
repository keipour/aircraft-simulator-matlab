classdef state_collection
    %STATE_COLLECTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(SetAccess=protected, GetAccess=public)
        States
        Size
        Capacity
    end
    
    methods
        function obj = state_collection()
            obj = obj.Reset();
        end
        
        function obj = SetCapacity(obj, value)
            if value == 0
                obj.States = {};
                obj.Size = 0;
            elseif obj.Size >= value
                obj.States = obj.States{1:value, 1};
                obj.Size = value;
            elseif obj.Capacity >= value
                obj.States = obj.States{1:value, 1};
            else 
                obj.States{value, 1} = state;
            end
            obj.Capacity = value;
        end
        
        function flag = IsEmpty(obj)
            flag = obj.Size > 0;
        end
        
        function obj = Reset(obj)
            obj = obj.SetCapacity(0);
        end
        
        function obj = PushBack(obj, new_state)
            obj.Size = obj.Size + 1;
            obj.States{obj.Size, 1} = new_state;
            if obj.Capacity < obj.Size
                obj.Capacity = obj.Size;
            end
        end
        
        function obj = SetStates(obj, value)
            obj.States = value;
            obj.Size = value;
            obj.Capacity = value;
        end
    end
end

