classdef state_collection < handle
    %STATE_COLLECTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(SetAccess=protected, GetAccess=public)
        States
        Size
        Capacity
    end
    
    methods
        function obj = state_collection()
            obj.Reset();
        end
        
        function SetCapacity(obj, value)
            if value == 0
                obj.States = {};
                obj.Size = 0;
            elseif obj.Size >= value
                obj.States = obj.States{1:value, 1};
                obj.Size = value;
            elseif obj.Capacity >= value
                obj.States = obj.States{1:value, 1};
            else 
                obj.States{value, 1} = state.Create();
            end
            obj.Capacity = value;
        end
        
        function flag = IsEmpty(obj)
            flag = obj.Size > 0;
        end
        
        function Reset(obj)
            obj.SetCapacity(0);
        end
        
        function PushBack(obj, new_state)
            obj.Size = obj.Size + 1;
            obj.States{obj.Size, 1} = new_state;
            if obj.Capacity < obj.Size
                obj.Capacity = obj.Size;
            end
        end
        
        function SetStates(obj, states)
            obj.States = states;
            obj.Size = length(states);
            obj.Capacity = length(states);
        end
    end
end