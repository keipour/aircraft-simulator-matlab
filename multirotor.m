classdef multirotor
    %PLANT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Fixed Properties
        Rotors rotor
        Mass = 3.0;                 % in Kg
        Gravity = [0; 0; 9.80665];  % in m/s^2
        I                           % Inertia
        PayloadRadius = 0.15;       % in meters
    end

    properties(SetAccess=protected, GetAccess=public)
        NumOfRotors                 % Number of rotors
        InitialState state          % Initial state
        State state                 % The current state
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
            
            % The number of rotors in the multirotor
            obj.NumOfRotors = length(ArmAngles);
            
            % Create the array of rotors
            obj.Rotors(obj.NumOfRotors, 1) = rotor;
            
            for i = 1 : obj.NumOfRotors
                obj.Rotors(i).ArmAngle = ArmAngles(i);
                obj.Rotors(i).RotationDirection = RotationDirections(i);
            end
            
            obj.InitialState = state();
            obj.State = state();
            
            obj = obj.UpdateStructure();
        end
        
        function obj = set.Rotors(obj, value)
            obj.Rotors = value;
            obj.UpdateNumOfRotors();
        end
        
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
            
            % Update the structure
            obj = obj.UpdateStructure();
        end
        
        function obj = UpdateState(obj, RotorSpeedsSquared, dt)
            % Calculate the total force and moment
            obj.State.Force = obj.GetGravityForce() + ...
                obj.GetThrustForce(RotorSpeedsSquared);
            obj.State.Moment = obj.GetGravityMoment() + ...
                obj.GetThrustMoment(RotorSpeedsSquared) + ...
                obj.GetReactionMoment(RotorSpeedsSquared);
            
            % Calculate the equations of motion
            p_dotdot = obj.GetLinearAcceleration(obj.State.Force);
            omega_dot = obj.GetAngularAcceleration(obj.State.Moment);
            phi_dot = obj.GetEulerDerivative();
            
            % Update the rest of the state
            obj.State.Position = obj.State.Position + 0.5 * obj.State.Acceleration * dt * dt + ...
                obj.State.Velocity * dt;
            obj.State.Velocity = obj.State.Velocity + obj.State.Acceleration * dt;
            obj.State.Acceleration = p_dotdot;

            obj.State.RPY = wrapTo180(obj.State.RPY + obj.State.EulerDerivative * dt);
            obj.State.Omega = obj.State.Omega + obj.State.AngularAcceleration * dt;
            obj.State.EulerDerivative = phi_dot;
            obj.State.AngularAcceleration = omega_dot;
        end
        
        function obj = set.I(obj, value)
            obj.I = value;
            obj = obj.UpdateI_inv();
        end
        
        function obj = UpdateStructure(obj)
            obj.I = calc_inertia(obj);
            obj.UpdateNumOfRotors();
        end
       
    end
    
    %% Private Methods
    methods(Access=protected)
        
        function obj = UpdateNumOfRotors(obj)
            obj.NumOfRotors = length(obj.Rotors);
        end
        
        function obj = UpdateI_inv(obj)
            obj.I_inv = pinv(obj.I);
        end

        function F = GetGravityForce(obj)
            F = obj.Gravity * obj.Mass;
        end
        
        function F = GetThrustForce(obj, RotorSpeedsSquared)
            F = zeros(3, 1);
            for i = 1 : obj.NumOfRotors
               F = F + obj.Rotors(i).GetThrustForce(RotorSpeedsSquared(i));
            end
        end
        
        function M = GetGravityMoment(obj)
            M = zeros(3, 1);
            for i = 1 : obj.NumOfRotors
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
            for i = 1 : obj.NumOfRotors
                r = obj.Rotors(i).Position;
                F = obj.Rotors(i).GetThrustForce(RotorSpeedsSquared(i));
                M = M + cross(r, F);
            end
        end
        
        function M = GetReactionMoment(obj, RotorSpeedsSquared)
            M = zeros(3, 1);
            for i = 1 : obj.NumOfRotors
               M = M + obj.Rotors(i).GetReactionMoment(RotorSpeedsSquared(i));
            end
        end
        
        function p_dotdot = GetLinearAcceleration(obj, force)
            p_dotdot = force / obj.Mass;
        end
        
        function omega_dot = GetAngularAcceleration(obj, moment)
            omega_dot = obj.I_inv * (moment - cross(obj.State.Omega, obj.I * obj.State.Omega));
        end
        
        function phi_dot = GetEulerDerivative(obj)
            sphi = sind(obj.State.RPY(1));
            cphi = cosd(obj.State.RPY(1));
            ttheta = tand(obj.State.RPY(2));
            ctheta = cosd(obj.State.RPY(2));
            eta = [1,   sphi*ttheta, cphi*ttheta;
                   0, cphi, -sphi;
                   0, sphi / ctheta, cphi / ctheta];
            phi_dot = rad2deg(eta * obj.State.Omega);
        end
    end
end

