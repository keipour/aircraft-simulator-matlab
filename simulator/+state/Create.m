function s = Create(n_rotors)
    s.Acceleration        = zeros(3, 1);        % Linear acceleration
    s.EulerRate           = zeros(3, 1);        % The derivatives of RPY
    s.AngularAcceleration = zeros(3, 1);        % Angular acceleration
    s.Position            = zeros(3, 1);        % Position in global frame
    s.Velocity            = zeros(3, 1);        % Linear velocity
    s.RPY                 = zeros(3, 1);        % Roll/Pitch/Yaw
    s.Omega               = zeros(3, 1);        % Angular velocity
    s.Force               = zeros(3, 1);        % Total generated force
    s.Moment              = zeros(3, 1);        % Total generated moment
    s.RotorSpeeds         = zeros(n_rotors, 1); % Rotor speeds
    s.RotorsSaturated     = false;              % If rotor saturation happens

    s.EndEffectorPosition = zeros(3, 1);        % End effector position in global frame
    s.EndEffectorVelocity = zeros(3, 1);        % End effector velocity in global frame
    s.EndEffectorOmega    = zeros(3, 1);        % End effector angular velocity in global frame

    s.ForceSensor         = zeros(3, 1);        % Force sensor reading in N
    s.MomentSensor        = zeros(3, 1);        % Moment sensor reading in N/m
    
    s.InCollision         = false;              % If multirotor is in collision
end
