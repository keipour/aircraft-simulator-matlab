function obj = Set(obj, states)
    obj.States = states;
    obj.Size = length(states);
    obj.Capacity = length(states);
end
