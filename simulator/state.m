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
        
        ServoAngles         = [];                 % Servo angles (in degrees)
        RotorInwardAngles   = [];                 % Rotor inward angles (in degrees)
        RotorSidewardAngles = [];                 % Rotor sideward angles (in degrees)
        TiltAngle           = 0;                  % Robot's tilt angle (in degrees)
        TiltDirection       = 0;                  % Tilt direction w.r.t. north (in degrees)
        Aileron             = 0;                  % Angle of aileron (in degree)
        AileronRate         = 0;                  % Derivative of aileron
        
        AirVelocity         = zeros(3, 1);        % 3-D airspeed vector (in m/s)
        AngleOfAttack       = 0;                  % in degrees (used for vtol)
        SideSlipAngle       = 0;                  % in degrees (used for vtol)
        
        AileronLeftPos      = 0;                  % Value in [-1..1] range
        AileronRightPos     = 0;                  % Value in [-1..1] range
        RudderPos           = 0;                  % Value in [-1..1] range
        ElevatorPos         = 0;                  % Value in [-1..1] range
        
        AileronLeftRate     = 0;
        AileronRightRate    = 0;
        RudderRate          = 0;
        ElevatorRate        = 0;
    end
    
    methods
        function obj = state(n_rotors, n_servos)
            if nargin > 0
                obj.RotorSpeeds = zeros(n_rotors, 1);
                obj.RotorInwardAngles = zeros(n_rotors, 1);
                obj.RotorSidewardAngles = zeros(n_rotors, 1);
            end
            if nargin > 1
                obj.ServoAngles = zeros(n_servos, 1);
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
            obj.ServoAngles         = s.ServoAngles;
            obj.RotorInwardAngles   = s.RotorInwardAngles;
            obj.RotorSidewardAngles = s.RotorSidewardAngles;
            obj.RotorsSaturated     = s.RotorsSaturated;

            obj.EndEffectorPosition = s.EndEffectorPosition;
            obj.EndEffectorVelocity = s.EndEffectorVelocity;
            obj.EndEffectorOmega    = s.EndEffectorOmega;

            obj.ForceSensor         = s.ForceSensor;
            obj.MomentSensor        = s.MomentSensor;

            obj.WindForce           = s.WindForce;
            obj.InCollision         = s.InCollision;
            
            obj.TiltAngle           = s.TiltAngle;
            obj.TiltDirection       = s.TiltDirection;

            obj.AirVelocity         = s.AirVelocity;
            obj.AngleOfAttack       = s.AngleOfAttack;
            obj.SideSlipAngle       = s.SideSlipAngle;
            
            obj.AileronLeftPos      = s.AileronLeftPos;
            obj.AileronRightPos     = s.AileronRightPos;
            obj.RudderPos           = s.RudderPos;
            obj.ElevatorPos         = s.ElevatorPos;

            obj.AileronLeftRate     = s.AileronLeftRate;
            obj.AileronRightRate    = s.AileronRightRate;
            obj.RudderRate          = s.RudderRate;
            obj.ElevatorRate        = s.ElevatorRate;
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
            s.ServoAngles         = obj.ServoAngles;
            s.RotorInwardAngles   = obj.RotorInwardAngles;
            s.RotorSidewardAngles = obj.RotorSidewardAngles;
            s.RotorsSaturated     = obj.RotorsSaturated;
            
            s.EndEffectorPosition = obj.EndEffectorPosition;
            s.EndEffectorVelocity = obj.EndEffectorVelocity;
            s.EndEffectorOmega    = obj.EndEffectorOmega;

            s.ForceSensor         = obj.ForceSensor;
            s.MomentSensor        = obj.MomentSensor;

            s.WindForce           = obj.WindForce;
            s.InCollision         = obj.InCollision;
            
            s.TiltAngle           = obj.TiltAngle;
            s.TiltDirection       = obj.TiltDirection;

            s.AirVelocity         = obj.AirVelocity;
            s.AngleOfAttack       = obj.AngleOfAttack;
            s.SideSlipAngle       = obj.SideSlipAngle;

            s.AileronLeftPos      = obj.AileronLeftPos;
            s.AileronRightPos     = obj.AileronRightPos;
            s.RudderPos           = obj.RudderPos;
            s.ElevatorPos         = obj.ElevatorPos;

            s.AileronLeftRate     = obj.AileronLeftRate;
            s.AileronRightRate    = obj.AileronRightRate;
            s.RudderRate          = obj.RudderRate;
            s.ElevatorRate        = obj.ElevatorRate;            
        end
    end
end

