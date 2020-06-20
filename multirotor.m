classdef multirotor
    %PLANT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Fixed Properties
        Rotors rotor
        Mass = 3.0;                 % in Kg
        Gravity = [0; 0; 9.80665];  % in m/s^2
        I                           % Inertia
        
        % State Variables
        Position = [0; 0; 0];       % [x, y, z] in meters
        Velocity = [0; 0; 0];       % [x_dot, y_dot, z_dot] in meters
        RPY = [0; 0; 0];            % [phi, theta, psi] : in degrees
        Omega = [0; 0; 0];          % [p, q, r] in rad/s
    end

    properties(SetAccess=protected, GetAccess=public)
        R                           % Rotation matrix RBI
        Force                       % Total generated force
        Moment                      % Total generated moment
        Acceleration = zeros(3, 1); % Linear acceleration
        EulerDerivative = zeros(3, 1); % Derivative of Euler angles
        AngularAcceleration = zeros(3, 1); % Angular acceleration
    end
    
    properties(SetAccess=protected, GetAccess=protected)
        I_inv                       % Inversion of I
    end
    
    %% Public methods
    methods
        function obj = multirotor(ArmAngles, RotationDirections)
            % Constructor for the multirotor class
            % The number of rotors is obtained from the length of the
            % ArmAngles input.
            
            % Convert the Euler angles to the rotation matrix
            obj.R = obj.GetRotationMatrix();
            
            % The number of rotors in the multirotor
            num_of_rotors = length(ArmAngles);
            
            % Create the array of rotors
            obj.Rotors(num_of_rotors, 1) = rotor;
            
            for i = 1 : num_of_rotors
                obj.Rotors(i).ArmAngle = ArmAngles(i);
                obj.Rotors(i).RotationDirection = RotationDirections(i);
            end
        end
        
        function n = GetNumOfRotors(obj)
            n = length(obj.Rotors);
        end
        
        function obj = SetRotorAngles(obj, RotorInwardAngles, RotorSidewardAngles, RotorDihedralAngles)
            % Set the angles of the rotors
            % Inputs can be scalar or an array of he same length as 
            % the number of rotors.

            % Some initialization
            num_of_rotors = length(obj.Rotors);

            if length(RotorInwardAngles) == 1
                RotorInwardAngles = ones(num_of_rotors, 1) * RotorInwardAngles;
            end
            if length(RotorSidewardAngles) == 1
                RotorSidewardAngles = ones(num_of_rotors, 1) * RotorSidewardAngles;
            end
            if length(RotorDihedralAngles) == 1
                RotorDihedralAngles = ones(num_of_rotors, 1) * RotorDihedralAngles;
            end
            
            % Assign the values
            for i = 1 : num_of_rotors
                obj.Rotors(i).InwardAngle = RotorInwardAngles(i);
                obj.Rotors(i).SidewardAngle = RotorSidewardAngles(i);
                obj.Rotors(i).DihedralAngle = RotorDihedralAngles(i);
            end
        end
        
        function R = GetRotationMatrix(obj)
            roll = deg2rad(obj.RPY(1));
            pitch = deg2rad(obj.RPY(2));
            yaw = deg2rad(obj.RPY(3));
            R = angle2dcm(yaw, pitch, roll);
        end
        
        function obj = UpdateState(obj, RotorSpeedsSquared, dt)
            obj.Force = obj.GetGravityForce() + ...
                obj.GetThrustForce(RotorSpeedsSquared);
            obj.Moment = obj.GetGravityMoment() + ...
                obj.GetThrustMoment(RotorSpeedsSquared) + ...
                obj.GetReactionMoment(RotorSpeedsSquared);
            
            % Calculate the equations of motion
            p_dotdot = obj.GetLinearAcceleration(obj.Force);
            omega_dot = obj.GetAngularAcceleration(obj.Moment);
            phi_dot = obj.GetEulerDerivative();
            
            % Update the state
            obj.Acceleration = p_dotdot;
            obj.EulerDerivative = phi_dot;
            obj.AngularAcceleration = omega_dot;
            obj.Position = obj.Position + 0.5 * p_dotdot * dt * dt + obj.Velocity * dt;
            obj.Velocity = obj.Velocity + p_dotdot * dt;
            obj.RPY = wrapTo180(obj.RPY + phi_dot * dt);
            obj.Omega = obj.Omega + omega_dot * dt;
        end
        
        function obj = set.I(obj, value)
            obj.I = value;
            obj = obj.UpdateI_inv();
        end
        
        function X = GetState(obj)
            X.Position = obj.Position;
            X.Velocity = obj.Velocity;
            X.RPY = obj.RPY;
            X.Omega = obj.Omega;
            X.Acceleration = obj.Acceleration;
            X.AngularAcceleration = obj.AngularAcceleration;
            X.EulerDerivative = obj.EulerDerivative;
        end
    end    
    
    %% Private Methods
    methods(Access=protected)
        
        function obj = UpdateI_inv(obj)
            obj.I_inv = pinv(obj.I);
        end

        function F = GetGravityForce(obj)
            F = obj.Gravity * obj.Mass;
        end
        
        function F = GetThrustForce(obj, RotorSpeedsSquared)
            F = zeros(3, 1);
            for i = 1 : obj.GetNumOfRotors()
               F = F + obj.Rotors(i).GetThrustForce(RotorSpeedsSquared(i));
            end
        end
        
        function M = GetGravityMoment(obj)
            M = zeros(3, 1);
            for i = 1 : obj.GetNumOfRotors()
                r = obj.Rotors(i).Position;
                G_motor = obj.Rotors(i).MotorMass * obj.Gravity;
                G_motorB = obj.Rotors(i).R * G_motor;
                G_arm = obj.Rotors(i).ArmMass * obj.Gravity;
                G_armB = obj.Rotors(i).R * G_arm;
                M = M + cross(r, G_motorB) + cross(r/2, G_armB);
            end
        end
                
        function M = GetThrustMoment(obj, RotorSpeedsSquared)
            M = zeros(3, 1);
            for i = 1 : obj.GetNumOfRotors()
                r = obj.Rotors(i).Position;
                F = obj.Rotors(i).GetThrustForce(RotorSpeedsSquared(i));
                M = M + cross(r, F);
            end
        end
        
        function M = GetReactionMoment(obj, RotorSpeedsSquared)
            M = zeros(3, 1);
            for i = 1 : obj.GetNumOfRotors()
               M = M + obj.Rotors(i).GetReactionMoment(RotorSpeedsSquared(i));
            end
        end
        
        function p_dotdot = GetLinearAcceleration(obj, Force)
            p_dotdot = Force / obj.Mass;
        end
        
        function omega_dot = GetAngularAcceleration(obj, Moment)
            omega_dot = obj.I_inv * (Moment - cross(obj.Omega, obj.I * obj.Omega));
        end
        
        function phi_dot = GetEulerDerivative(obj)
            sphi = sind(obj.RPY(1));
            cphi = cosd(obj.RPY(1));
            ttheta = tand(obj.RPY(2));
            ctheta = cosd(obj.RPY(2));
            eta = [1,   sphi*ttheta, cphi*ttheta;
                   0, cphi, -sphi;
                   0, sphi / ctheta, cphi / ctheta];
            phi_dot = rad2deg(eta * obj.Omega);
        end
    end
end

