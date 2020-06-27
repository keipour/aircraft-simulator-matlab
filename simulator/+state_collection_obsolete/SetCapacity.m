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
        obj.States{value, 1} = state.Create();
    end
    obj.Capacity = value;
end