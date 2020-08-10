classdef multirotor < handle
    properties
        % Fixed Properties
        Rotors
        Mass = 7.427;                 % in Kg
        I                           % Inertia
        PayloadRadius = 0.15;       % in meters
        
        TotalSpeedLimit = 20;                       % in m/s
        VelocityLimits = [10; 10; 8];               % in m/s
        OmegaLimits = deg2rad([70; 70; 30]);        % in deg/s
    end

    properties(SetAccess=protected, GetAccess=public)
        NumOfRotors                 % Number of rotors
        TotalMass                   % Total mass including the arm
        InitialState                % Initial state
        State                       % The current state
        I_inv                       % Inversion of I
        EndEffector arm
    end
    
    properties(SetAccess=protected, GetAccess=protected)
        LastTime = 0;               % Last update time
        HasArm = false;
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
            obj.Rotors = cell(obj.NumOfRotors, 1);
            
            for i = 1 : obj.NumOfRotors
                obj.Rotors{i} = rotor.Create();
                obj.Rotors{i} = rotor.SetArmAngle(obj.Rotors{i}, ArmAngles(i));
                obj.Rotors{i}.RotationDirection = RotationDirections(i);
            end
            
            obj.InitialState = state.Create(obj.NumOfRotors);
            obj.State = state.Create(obj.NumOfRotors);
            
            obj.UpdateStructure();
        end
        
        function set.Rotors(obj, value)
            obj.Rotors = value;
            obj.UpdateNumOfRotors();
        end
        
        function AddEndEffector(obj, end_effector)
            if nargin < 2
                obj.HasArm = false;
                return;
            end
            obj.EndEffector = end_effector;
            obj.HasArm = true;
            obj.UpdateStructure();
        end
        
        function has_end_effector = HasEndEffector(obj)
            has_end_effector = obj.HasArm;
        end
        
        function e_pos = GetEndEffectorPosition(obj)
            obj.CheckEndEffector();
            RBI = obj.GetRotationMatrix();
            e_pos = state.Position + RBI' * obj.EndEffector.EndEffectorPosition;
        end
        
        function e_rot = GetEndEffectorRotation(obj)
            obj.CheckEndEffector();
            RBI = obj.GetRotationMatrix();
            e_rot = RBI' * obj.EndEffector.R_BR;
        end
        
        function SetInitialState(obj, pos, vel, rpy, omega)
            obj.InitialState.Position = pos;
            obj.InitialState.Velocity = vel;
            obj.InitialState.RPY = rpy;
            obj.InitialState.Omega = omega;
            
            obj.State.Position = pos;
            obj.State.Velocity = vel;
            obj.State.RPY = rpy;
            obj.State.Omega = omega;
            
            obj.LastTime = 0;
        end
        
        function SetRotorAngles(obj, RotorInwardAngles, RotorSidewardAngles, RotorDihedralAngles)
            % Set the angles of the rotors
            % Inputs can be scalar or an array of he same length as 
            % the number of rotors.

            if length(RotorInwardAngles) == 1
                RotorInwardAngles = ones(obj.NumOfRotors, 1) * RotorInwardAngles;
            end
            if length(RotorSidewardAngles) == 1
                RotorSidewardAngles = ones(obj.NumOfRotors, 1) * RotorSidewardAngles;
            end
            if length(RotorDihedralAngles) == 1
                RotorDihedralAngles = ones(obj.NumOfRotors, 1) * RotorDihedralAngles;
            end
            
            % Assign the values
            for i = 1 : obj.NumOfRotors
                obj.Rotors{i} = rotor.SetInwardAngle(obj.Rotors{i}, RotorInwardAngles(i));
                obj.Rotors{i} = rotor.SetSidewardAngle(obj.Rotors{i}, RotorSidewardAngles(i));
                obj.Rotors{i} = rotor.SetDihedralAngle(obj.Rotors{i}, RotorDihedralAngles(i));
            end
            
            % Update the structure
            obj.UpdateStructure();
        end
        
        function UpdateState(obj, RotorSpeedsSquared, time)
            % Calculate the time step
            dt = time - obj.LastTime;
            
            obj.UpdateStateNewtonEuler(RotorSpeedsSquared, dt);
            
            % Update the time of the update
            obj.LastTime = time;
        end
        
        function accel = CalculateAccelerationManipulability(obj, RotorSpeedsSquared, get_maximum)
            if nargin < 3
                get_maximum = false;
            end
            accel = (obj.GetGravityForce() + ...
                obj.GetThrustForce(eye(3), RotorSpeedsSquared, get_maximum)) / obj.TotalMass;
        end
        
        function omega_dot = CalculateAngularAccelerationManipulability(obj, RotorSpeedsSquared)
            moment = obj.GetGravityMoment(eye(3)) + obj.GetThrustMoment(RotorSpeedsSquared) + ...
                obj.GetReactionMoment(RotorSpeedsSquared);
            omega_dot = obj.GetAngularAcceleration(moment);
        end
        
        function set.I(obj, value)
            obj.I = value;
            obj.UpdateI_inv();
        end
        
        function UpdateStructure(obj)
            obj.I = obj.EstimateInertia();
            obj.UpdateNumOfRotors();
            obj.LastTime = 0;
            obj.TotalMass = obj.Mass;
            if obj.HasArm
                obj.TotalMass = obj.TotalMass + obj.EndEffector.TotalMass;
            end
        end
        
        function inertia_tensor = EstimateInertia(obj)
        % Estimate the inertia tensor of the multirotor
            inertia_tensor = physics.EstimateInertia(obj);
        end
        
        function CopyFrom(obj, mult)
            obj.Rotors = mult.Rotors;
            obj.Mass = mult.Mass;
            obj.I = mult.I;
            obj.PayloadRadius = mult.PayloadRadius;
            obj.NumOfRotors = mult.NumOfRotors;
            obj.InitialState = mult.InitialState;
            obj.State = mult.State;
            obj.I_inv = mult.I_inv;
            obj.LastTime = 0;
            obj.HasArm = mult.HasEndEffector();
            obj.TotalSpeedLimit = mult.TotalSpeedLimit;
            obj.VelocityLimits = mult.VelocityLimits;
            obj.OmegaLimits = mult.OmegaLimits;
            obj.TotalMass = mult.TotalMass;
            if obj.HasArm == true
                obj.EndEffector = arm;
                obj.EndEffector.CopyFrom(mult.EndEffector);
            end
        end
        
        function Visualize(obj)
            graphics.VisualizeMultirotor(obj, false);
        end
        
        function VisualizeAxes(obj)
            graphics.VisualizeMultirotor(obj, true);
        end
        
        function AnalyzeDynamicManipulability(obj, lin_steps, ang_steps)
            analysis.AnalyzeDynamicManipulability(obj, lin_steps, ang_steps);
        end
        
        function R = GetRotationMatrix(obj)
            roll = deg2rad(obj.State.RPY(1));
            pitch = deg2rad(obj.State.RPY(2));
            yaw = deg2rad(obj.State.RPY(3));
            R = angle2dcm(yaw, pitch, roll);
        end
    end
    
    %% Private Methods
    methods(Access=protected)
        
        function UpdateStateNewtonEuler(obj, RotorSpeedsSquared, dt)
            % Calculate the current rotation matrix
            RBI = obj.GetRotationMatrix();
            
            % Calculate the total force and moment
            obj.State.Force = obj.GetGravityForce() + ...
                obj.GetThrustForce(RBI', RotorSpeedsSquared);
            obj.State.Moment = obj.GetGravityMoment(RBI) + ...
                obj.GetThrustMoment(RotorSpeedsSquared) + ...
                obj.GetReactionMoment(RotorSpeedsSquared);
            
            % Calculate the equations of motion
            p_dotdot = obj.GetLinearAcceleration(obj.State.Force);
            omega_dot = obj.GetAngularAcceleration(obj.State.Moment);
            phi_dot = obj.GetEulerRate();
            
            % Update the rest of the state
            obj.State.Position = obj.State.Position + 0.5 * obj.State.Acceleration * dt * dt + ...
                obj.State.Velocity * dt;
            obj.State.Velocity = obj.State.Velocity + obj.State.Acceleration * dt;
            obj.State.Velocity = check_limits(obj.State.Velocity, obj.VelocityLimits);
            obj.State.Velocity = check_limits(obj.State.Velocity, obj.TotalSpeedLimit);
            obj.State.Acceleration = p_dotdot;

            obj.State.RPY = wrapTo180(obj.State.RPY + obj.State.EulerRate * dt);
            obj.State.Omega = obj.State.Omega + obj.State.AngularAcceleration * dt;
            obj.State.Omega = check_limits(obj.State.Omega, obj.OmegaLimits);

            obj.State.EulerRate = phi_dot;
            obj.State.AngularAcceleration = omega_dot;
            
            for i = 1 : obj.NumOfRotors
                [rs, sat] = rotor.LimitRotorSpeed(obj.Rotors{i}, RotorSpeedsSquared(i));
                obj.State.RotorSpeeds(i) = sqrt(rs);
                obj.State.RotorsSaturated = obj.State.RotorsSaturated || sat;
            end
        end
        
        function UpdateStateEulerLagrange(obj, RotorSpeedsSquared, dt)

        end
        
        function UpdateNumOfRotors(obj)
            obj.NumOfRotors = length(obj.Rotors);
        end
        
        function UpdateI_inv(obj)
            obj.I_inv = pinv(obj.I);
        end

        function F = GetGravityForce(obj)
            F = physics.Gravity * obj.TotalMass;
        end
        
        function F = GetThrustForce(obj, Rot_IB, RotorSpeedsSquared, get_maximum)
            FB = zeros(3, 1);
            for i = 1 : obj.NumOfRotors
                if nargin < 4 || get_maximum == false
                    FB = FB + rotor.GetThrustForce(obj.Rotors{i}, RotorSpeedsSquared(i));
                else
                    max_thrust = [0; 0; -norm(rotor.GetThrustForce(obj.Rotors{i}, obj.Rotors{i}.MaxrotorSpeedSquared))];
                    FB = FB + max_thrust;
                end
            end
            F = Rot_IB * FB;
        end
        
        function M = GetGravityMoment(obj, Rot_BI)
            M = zeros(3, 1);
            for i = 1 : obj.NumOfRotors
                r = obj.Rotors{i}.Position;
                G_motorI = obj.Rotors{i}.MotorMass * physics.Gravity;
                G_motorB = Rot_BI * G_motorI;
                G_armI = obj.Rotors{i}.ArmMass * physics.Gravity;
                G_armB = Rot_BI * G_armI;
                M = M + cross(r, G_motorB) + cross(r/2, G_armB);
            end
            if obj.HasArm
                r = obj.EndEffector.EndEffectorPosition;
                G_eeI = obj.EndEffector.EndEffectorMass * physics.Gravity;
                G_eeB = Rot_BI * G_eeI;
                G_armI = obj.EndEffector.ArmMass * physics.Gravity;
                G_armB = Rot_BI * G_armI;
                M = M + cross(r, G_eeB) + cross(r/2, G_armB);
            end
        end
        
        function M = GetThrustMoment(obj, RotorSpeedsSquared)
            M = zeros(3, 1);
            for i = 1 : obj.NumOfRotors
                r = obj.Rotors{i}.Position;
                F = rotor.GetThrustForce(obj.Rotors{i}, RotorSpeedsSquared(i));
                M = M + cross(r, F);
            end
        end
        
        function M = GetReactionMoment(obj, RotorSpeedsSquared)
            M = zeros(3, 1);
            for i = 1 : obj.NumOfRotors
               M = M + rotor.GetReactionMoment(obj.Rotors{i}, RotorSpeedsSquared(i));
            end
        end
        
        function p_dotdot = GetLinearAcceleration(obj, force)
            p_dotdot = force / obj.TotalMass;
        end
        
        function omega_dot = GetAngularAcceleration(obj, moment)
            omega_dot = obj.I_inv * (moment - cross(obj.State.Omega, obj.I * obj.State.Omega));
        end
        
        function phi_dot = GetEulerRate(obj)
        % Returns the euler rates in degrees   
            
            sphi = sind(obj.State.RPY(1));
            cphi = cosd(obj.State.RPY(1));
            ttheta = tand(obj.State.RPY(2));
            ctheta = cosd(obj.State.RPY(2));
            eta = [1,   sphi*ttheta, cphi*ttheta;
                   0, cphi, -sphi;
                   0, sphi / ctheta, cphi / ctheta];
            phi_dot = rad2deg(eta * obj.State.Omega);
        end
        
        function CheckEndEffector(obj)
            if ~obj.HasArm
                error('End-effector is not defined for this multirotor.');
            end
        end
    end
end

%% Other functions
function x_lim = check_scalar_limit(x, lim)
    x_lim = min(x, lim);
    if x < 0
        x_lim = max(x, -lim);
    end
end

function x_lim = check_limits(x, lim)
    x_lim = x;
    
    if length(lim) == 1
        nx = norm(x);
        if nx > lim
            x_lim = x / nx * lim;
        end
        return;
    end

    for i = 1 : length(x)
        x_lim(i) = check_scalar_limit(x(i), lim(i));
    end
end

function X = skewsym(x)
    X = [0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
end
