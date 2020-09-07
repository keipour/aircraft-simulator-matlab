function s = CopyFrom(s, input_state)
    s.Acceleration        = input_state.Acceleration;
    s.EulerRate           = input_state.EulerRate;
    s.AngularAcceleration = input_state.AngularAcceleration;
    s.Position            = input_state.Position;
    s.Velocity            = input_state.Velocity;
    s.RPY                 = input_state.RPY;
    s.Omega               = input_state.Omega;
    s.Force               = input_state.Force;
    s.Moment              = input_state.Moment;
    s.RotorSpeeds         = input_state.RotorSpeeds;
    s.RotorsSaturated     = input_state.RotorsSaturated;
    
    s.EndEffectorPosition = input_state.EndEffectorPosition;
    s.EndEffectorVelocity = input_state.EndEffectorVelocity;
    s.EndEffectorOmega    = input_state.EndEffectorOmega;
    
    s.ForceSensor         = input_state.ForceSensor;
    s.MomentSensor        = input_state.MomentSensor;
    
    s.WindForce           = input_state.WindForce;
    
    s.InCollision         = input_state.InCollision;
end
