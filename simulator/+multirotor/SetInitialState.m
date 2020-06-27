function obj = SetInitialState(obj, pos, vel, rpy, omega)
    obj.InitialState.Position = pos;
    obj.InitialState.Velocity = vel;
    obj.InitialState.RPY = rpy;
    obj.InitialState.Omega = omega;

    obj.State.Position = pos;
    obj.State.Velocity = vel;
    obj.State.RPY = rpy;
    obj.State.Omega = omega;
end
