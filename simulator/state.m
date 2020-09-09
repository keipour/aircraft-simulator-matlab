classdef state < handle

    properties
        Acceleration        = zeros(3, 1);        % Linear acceleration
        EulerRate           = zeros(3, 1);        % The derivatives of RPY
        AngularAcceleration = zeros(3, 1);        % Angular acceleration
        Position            = zeros(3, 1);        % Position in global frame
        Velocity            = zeros(3, 1);        % Linear velocity
        RPY                 = zeros(3, 1);        % Roll/Pitch/Yaw
        Omega               = zeros(3, 1);        % Angular velocity
        Force               = zeros(3, 1);        % Total generated force
        Moment              = zeros(3, 1);        % Total generated moment
        RotorSpeeds         = [];                 % Rotor speeds
        RotorsSaturated     = false;              % If rotor saturation happens

        EndEffectorPosition = zeros(3, 1);        % End effector position in global frame
        EndEffectorVelocity = zeros(3, 1);        % End effector velocity in global frame
        EndEffectorOmega    = zeros(3, 1);        % End effector angular velocity in global frame

        ForceSensor         = zeros(3, 1);        % Force sensor reading in N
        MomentSensor        = zeros(3, 1);        % Moment sensor reading in N/m

        WindForce           = zeros(3, 1);        % Wind force applied to the CoM in N

        InCollision         = false;              % If multirotor is in collision
    end
    
    methods
        function obj = state(n_rotors)
            if nargin > 0
                obj.RotorSpeeds = zeros(n_rotors, 1);
            end
        end
        
        function CopyFrom(obj, s)
            obj.Acceleration        = s.Acceleration;
            obj.EulerRate           = s.EulerRate;
            obj.AngularAcceleration = s.AngularAcceleration;
            obj.Position            = s.Position;
            obj.Velocity            = s.Velocity;
            obj.RPY                 = s.RPY;
            obj.Omega               = s.Omega;
            obj.Force               = s.Force;
            obj.Moment              = s.Moment;
            obj.RotorSpeeds         = s.RotorSpeeds;
            obj.RotorsSaturated     = s.RotorsSaturated;

            obj.EndEffectorPosition = s.EndEffectorPosition;
            obj.EndEffectorVelocity = s.EndEffectorVelocity;
            obj.EndEffectorOmega    = s.EndEffectorOmega;

            obj.ForceSensor         = s.ForceSensor;
            obj.MomentSensor        = s.MomentSensor;

            obj.WindForce           = s.WindForce;

            obj.InCollision         = s.InCollision;
        end
        
        function s = struct(obj)
            s.Acceleration        = obj.Acceleration;
            s.EulerRate           = obj.EulerRate;
            s.AngularAcceleration = obj.AngularAcceleration;
            s.Position            = obj.Position;
            s.Velocity            = obj.Velocity;
            s.RPY                 = obj.RPY;
            s.Omega               = obj.Omega;
            s.Force               = obj.Force;
            s.Moment              = obj.Moment;
            s.RotorSpeeds         = obj.RotorSpeeds;
            s.RotorsSaturated     = obj.RotorsSaturated;

            s.EndEffectorPosition = obj.EndEffectorPosition;
            s.EndEffectorVelocity = obj.EndEffectorVelocity;
            s.EndEffectorOmega    = obj.EndEffectorOmega;

            s.ForceSensor         = obj.ForceSensor;
            s.MomentSensor        = obj.MomentSensor;

            s.WindForce           = obj.WindForce;

            s.InCollision         = obj.InCollision;
        end
    end
end

