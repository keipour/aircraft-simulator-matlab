function s = CopyFrom(s, new_state)
    s.Acceleration        = new_state.Acceleration;
    s.EulerRate           = new_state.EulerRate;
    s.AngularAcceleration = new_state.AngularAcceleration;
    s.Position            = new_state.Position;
    s.Velocity            = new_state.Velocity;
    s.RPY                 = new_stateRPY;
    s.Omega               = new_state.Omega;
    s.Force               = new_state.Force;
    s.Moment              = new_state.Moment;
    s.RotorSpeeds         = new_state.RotorSpeeds;
    s.RotorsSaturated     = new_state.RotorsSaturated;
end
