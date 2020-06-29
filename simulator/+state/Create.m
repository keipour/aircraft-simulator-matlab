function s = Create(n_rotors)
    s.Acceleration        = zeros(3, 1);        % Linear acceleration
    s.EulerDerivative     = zeros(3, 1);        % The derivatives of RPY
    s.AngularAcceleration = zeros(3, 1);        % Angular acceleration
    s.Position            = zeros(3, 1);        % Position in global frame
    s.Velocity            = zeros(3, 1);        % Linear velocity
    s.RPY                 = zeros(3, 1);        % Roll/Pitch/Yaw
    s.Omega               = zeros(3, 1);        % Angular velocity
    s.Force               = zeros(3, 1);        % Total generated force
    s.Moment              = zeros(3, 1);        % Total generated moment
    s.RotorSpeeds         = zeros(n_rotors, 1); % Rotor speeds
    s.RotorsSaturated     = false;              % If rotor saturation happens
end
