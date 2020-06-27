function obj = PushBack(obj, new_state)
    obj.Size = obj.Size + 1;
    obj.States{obj.Size, 1} = new_state;
    if obj.Capacity < obj.Size
        obj.Capacity = obj.Size;
    end
end
