function p_dotdot = GetLinearAcceleration(obj, force)
    p_dotdot = force / obj.Mass;
end
