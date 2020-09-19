classdef multirotor < handle
    properties
        Rotors cell
        Mass = 7.427;               % in Kg
        I                           % Inertia
        PayloadRadius = 0.15;       % in meters
        
        TotalSpeedLimit = 20;                          % in m/s
        VelocityLimits = [10; 10; 8];                  % in m/s
        OmegaLimits = deg2rad([70; 70; 30]);           % in deg/s
        WindModel support_files.multirotor_wind_model ...
            = support_files.multirotor_wind_model;     % Model used for wind pressure calculation
    end

    properties (SetAccess=protected, GetAccess=public)
        NumOfRotors                 % Number of rotors
        TotalMass                   % Total mass including the arm
        InitialState state          % Initial state
        State state                 % The current state
        I_inv                       % Inversion of I
        EndEffector arm             % Manipulator arm
        CollisionModel              % Model used for collision detection
        DynamicsMethod multirotor_dynamics_methods % The dynamics formulations
    end
    
    properties (SetAccess=protected, GetAccess=protected)
        HasArm = false;
        TransformedCollisionModels

        NE_L                        % L matrix in Newton-Euler dynamics formulation
                                    % (related to body-fixed thrust forces)

        NE_M                        % M matrix in Newton-Euler dynamics formulation
                                    % (related to body-fixed thrust and reaction moments)
                                    
        Ang_Acc_Manip_A             % The fixed part of the angular manipulability (A + Bu)
        Ang_Acc_Manip_B             % The part dependent on rotor speeds of the angular manipulability
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
                obj.Rotors{i} = rotor();
                obj.Rotors{i}.ArmAngle = ArmAngles(i);
                obj.Rotors{i}.RotationDirection = RotationDirections(i);
            end
            
            obj.InitialState = state(obj.NumOfRotors);
            obj.State = state(obj.NumOfRotors);
            
            % Set the dynamics formulation type
            obj.DynamicsMethod = multirotor_dynamics_methods.NewtonEuler;
            
            % Update the structure
            obj.UpdateStructure();
        end
        
        function SetDynamicsMethod(obj, method)
            obj.DynamicsMethod = method;
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
        
        function SetInitialState(obj, pos, vel, rpy, omega)
            obj.InitialState.Position = pos;
            obj.InitialState.Velocity = vel;
            obj.InitialState.RPY = rpy;
            obj.InitialState.Omega = omega;
            
            obj.State.Position = pos;
            obj.State.Velocity = vel;
            obj.State.RPY = rpy;
            obj.State.Omega = omega;
            
            if obj.HasArm
                obj.State.EndEffectorPosition = obj.CalcEndEffectorPosition(obj.State.Position, obj.State.RPY);
                obj.State.EndEffectorVelocity = obj.CalcEndEffectorVelocity(obj.State.Velocity, obj.State.Omega, obj.State.RPY);
                obj.State.EndEffectorOmega = obj.State.Omega;
            end
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
                obj.Rotors{i}.InwardAngle = RotorInwardAngles(i);
                obj.Rotors{i}.SidewardAngle = RotorSidewardAngles(i);
                obj.Rotors{i}.DihedralAngle = RotorDihedralAngles(i);
            end
            
            % Update the structure
            obj.UpdateStructure();
        end
        
        function UpdateState(obj, new_state)
            obj.State = new_state;
        end
        
        function wrench = CalcGeneratedWrench(obj, rotor_speeds_squared)
            
            RBI = obj.GetRotationMatrix();
            
            F_x = [obj.GetGravityMoment(RBI) - cross(obj.State.Omega, obj.I * obj.State.Omega);
                   obj.TotalMass * physics.Gravity];
            
            J_x = [obj.NE_M; RBI' * obj.NE_L];
                        
            wrench = F_x + J_x * rotor_speeds_squared;
        end
        
        function new_state = CalcNextState(obj, wrench, tf_sensor_wrench, ...
                wind_force, RotorSpeedsSquared, dt, is_collision, collision_normal)

            ext_wrench = [zeros(3, 1); wind_force];
            if obj.HasArm
                ee_wrench_matrix = [obj.EndEffector.R_BE, ...
                    skewsym(obj.EndEffector.EndEffectorPosition) * obj.EndEffector.R_BE; ...
                    zeros(3), obj.GetRotationMatrix()' * obj.EndEffector.R_BE];

                ext_wrench = ext_wrench + ee_wrench_matrix * tf_sensor_wrench;
            end
            
            % Calculate the time step
            switch obj.DynamicsMethod
                case multirotor_dynamics_methods.NewtonEuler
                    new_state = obj.CalcStateNewtonEuler(wrench + ext_wrench, ...
                        dt, is_collision, collision_normal);
                otherwise
                    error('The specified dynamics method is not implemented yet.');
            end
            
            % Save the rest of the state variables not filled before
            new_state.Force = wrench(4:6);
            new_state.Moment = wrench(1:3);
            new_state.ForceSensor = tf_sensor_wrench(4:6);
            new_state.MomentSensor = tf_sensor_wrench(1:3);
            new_state.InCollision = is_collision;
            new_state.WindForce = wind_force;
            
            for i = 1 : obj.NumOfRotors
                [rs, sat] = obj.Rotors{i}.LimitRotorSpeed(RotorSpeedsSquared(i));
                new_state.RotorSpeeds(i) = sqrt(rs);
                new_state.RotorsSaturated = new_state.RotorsSaturated || sat;
            end
        end
        
        function accel = CalculateAccelerationManipulability(obj, rotor_speeds_squared, get_maximum)
            if nargin < 3
                get_maximum = false;
            end
            
            if get_maximum
                accel = (obj.GetGravityForce() + ...
                    obj.GetThrustForce(eye(3), rotor_speeds_squared, get_maximum)) / obj.TotalMass;
            else
                accel = (obj.TotalMass * physics.Gravity + obj.NE_L * rotor_speeds_squared) / obj.TotalMass;
            end
        end
        
        function omega_dot = CalculateAngularAccelerationManipulability(obj, rotor_speeds_squared)
            omega_dot = obj.Ang_Acc_Manip_A + obj.Ang_Acc_Manip_B * rotor_speeds_squared;
        end
        
        function set.I(obj, value)
            obj.I = value;
            obj.UpdateI_inv();
        end
        
        function UpdateStructure(obj)
            obj.I = obj.EstimateInertia();
            obj.UpdateNumOfRotors();
            obj.TotalMass = obj.Mass;
            obj.CollisionModel = obj.CalculateCollisionModel();
            mult_cm = support_files.collision_box(obj.CollisionModel.X, ...
                obj.CollisionModel.Y, obj.CollisionModel.Z);
            
            obj.WindModel.Update(obj);

            if obj.HasArm
                obj.TotalMass = obj.TotalMass + obj.EndEffector.TotalMass;
                arm_cm = support_files.collision_box(obj.EndEffector.CollisionModel.X, ...
                    obj.EndEffector.CollisionModel.Y,...
                    obj.EndEffector.CollisionModel.Z);
                obj.TransformedCollisionModels = {mult_cm, arm_cm};
            else
                obj.TransformedCollisionModels = {mult_cm};
            end
            
            obj.InitializeDynamicsMethod();
        end
        
        function inertia_tensor = EstimateInertia(obj)
        % Estimate the inertia tensor of the multirotor
            inertia_tensor = physics.EstimateInertia(obj);
        end
        
        function CopyFrom(obj, mult)
            obj.Rotors = cell(size(mult.Rotors));
            for i = 1 : mult.NumOfRotors
                obj.Rotors{i} = rotor(mult.Rotors{i});
            end
            obj.Mass = mult.Mass;
            obj.I = mult.I;
            obj.PayloadRadius = mult.PayloadRadius;
            obj.NumOfRotors = mult.NumOfRotors;
            obj.InitialState = state();
            obj.InitialState = mult.InitialState;
            obj.State = state();
            obj.State.CopyFrom(mult.State);
            obj.I_inv = mult.I_inv;
            obj.NE_L = mult.NE_L;
            obj.NE_M = mult.NE_M;
            obj.HasArm = mult.HasEndEffector();
            obj.TotalSpeedLimit = mult.TotalSpeedLimit;
            obj.VelocityLimits = mult.VelocityLimits;
            obj.OmegaLimits = mult.OmegaLimits;
            obj.TotalMass = mult.TotalMass;
            obj.CollisionModel = support_files.collision_box(mult.CollisionModel.X, ...
                mult.CollisionModel.Y, mult.CollisionModel.Z);
            obj.CollisionModel.Pose = mult.CollisionModel.Pose;

            mult_cm = support_files.collision_box(mult.CollisionModel.X, ...
                mult.CollisionModel.Y, mult.CollisionModel.Z);
            if obj.HasArm
                arm_cm = support_files.collision_box(mult.EndEffector.CollisionModel.X, ...
                    mult.EndEffector.CollisionModel.Y,...
                    mult.EndEffector.CollisionModel.Z);
                obj.TransformedCollisionModels = {mult_cm, arm_cm};

                obj.EndEffector = arm;
                obj.EndEffector.CopyFrom(mult.EndEffector);
            else
                obj.TransformedCollisionModels = {mult_cm};
            end
            
            obj.WindModel.CopyFrom(mult.WindModel);
        end
        
        function H = Visualize(obj, draw_collision_model)
            if nargin < 2
                draw_collision_model = false;
            end
            H = graphics.VisualizeMultirotor(obj, false, draw_collision_model);
        end
        
        function H = VisualizeAxes(obj)
            H = graphics.VisualizeMultirotor(obj, true);
        end
        
        function AnalyzeDynamicManipulability(obj, lin_steps, ang_steps)
            analysis.AnalyzeDynamicManipulability(obj, lin_steps, ang_steps);
        end
        
        function RBI = GetRotationMatrix(obj)
            roll = obj.State.RPY(1);
            pitch = obj.State.RPY(2);
            yaw = obj.State.RPY(3);
            RBI = physics.GetRotationMatrixDegrees(roll, pitch, yaw);
        end
        
        function cm = CalculateCollisionModel(obj)
            x = [-obj.PayloadRadius, obj.PayloadRadius];
            y = [-obj.PayloadRadius, obj.PayloadRadius];
            z = [-obj.PayloadRadius, obj.PayloadRadius];

            for i = 1 : obj.NumOfRotors
                x(1) = min(x(1), obj.Rotors{i}.Position(1));
                x(2) = max(x(2), obj.Rotors{i}.Position(1));
                y(1) = min(y(1), obj.Rotors{i}.Position(2));
                y(2) = max(y(2), obj.Rotors{i}.Position(2));
                z(1) = min(z(1), obj.Rotors{i}.Position(3));
                z(2) = max(z(2), obj.Rotors{i}.Position(3));
            end
            
            cm = support_files.collision_box(x(2) - x(1), y(2) - y(1), z(2) - z(1));
            T = trvec2tform([mean(x), mean(y), mean(z)]);
            cm.Pose = T;
        end
        
        function cms = GetTransformedCollisionModel(obj, pos, rpy_rad)
            cms = obj.TransformedCollisionModels;
            RBI = physics.GetRotationMatrixRadians(rpy_rad(1), rpy_rad(2), rpy_rad(3));
            T = [RBI', pos; 0, 0, 0, 1];
            cms{1}.Pose = T * obj.CollisionModel.Pose;
            if obj.HasArm
                cms{2}.Pose = T * obj.EndEffector.CollisionModel.Pose;
            end
        end
        
    end
    
    %% Private Methods
    methods(Access=protected)
        
        function new_state = CalcStateNewtonEuler(obj, wrench, dt, is_collision, collision_normal)
            
            persistent last_collision_status
            if isempty(last_collision_status)
                last_collision_status = false;
            end
            
            % Create the new state
            new_state = state(obj.NumOfRotors);
            
            p_dotdot = wrench(4 : 6) / obj.TotalMass;
            phi_dot = obj.GetEulerRate();
            omega_dot = obj.I_inv * wrench(1 : 3);
            
            % I assume approximately constant acceleration
            new_state.Acceleration = p_dotdot;
            new_state.AngularAcceleration = omega_dot;
            new_state.EulerRate = phi_dot;

            new_state.Velocity = obj.State.Velocity + obj.State.Acceleration * dt;
            new_state.Velocity = check_limits(new_state.Velocity, obj.VelocityLimits);
            new_state.Velocity = check_limits(new_state.Velocity, obj.TotalSpeedLimit);
            new_state.Omega = obj.State.Omega + obj.State.AngularAcceleration * dt;
            new_state.Omega = check_limits(new_state.Omega, obj.OmegaLimits);
            new_state.Position = obj.State.Position + 0.5 * obj.State.Acceleration * dt * dt + obj.State.Velocity * dt;
            new_state.RPY = wrapTo180(obj.State.RPY + obj.State.EulerRate * dt);

            % Update the end effector's state
            if obj.HasArm
                new_state.EndEffectorPosition = obj.CalcEndEffectorPosition(new_state.Position, new_state.RPY);
                new_state.EndEffectorVelocity = obj.CalcEndEffectorVelocity(new_state.Velocity, new_state.Omega, new_state.RPY);
                new_state.EndEffectorOmega = new_state.Omega;
            end
            
            % If there is a collision
            if is_collision && ~last_collision_status
                % I assume no bouncing due to the impact and no impulse
                % TODO: Model based on https://www.sciencedirect.com/science/article/abs/pii/S0094114X02000459
                % We assume that the collision happened at the end effector
                % if it has end effector
                                
                if obj.HasArm
                    twist = [new_state.EndEffectorOmega; new_state.EndEffectorVelocity];
                    contact_twist = physics.ApplyContactConstraints...
                        (twist, collision_normal, diag([1, 1, 1, 0, 1, 1]), ...
                        [0; 0; 0; 1; 0; 0], [obj.GetRotationMatrix()'; eye(3)]);
                    new_state.EndEffectorOmega = contact_twist(1:3);
                    new_state.EndEffectorVelocity = contact_twist(4:6);
                    new_state.Velocity = obj.CalcRobotVelocityFromEndEffector(new_state.EndEffectorVelocity, new_state.EndEffectorOmega, new_state.RPY);
                    new_state.Omega = new_state.EndEffectorOmega;
%                     new_state.Acceleration = (new_state.Velocity - obj.State.Velocity) / dt;
%                     new_state.AngularAcceleration = (new_state.Omega - obj.State.Omega) / dt;
                else
                    new_state.Velocity = obj.State.Velocity + new_state.Acceleration * dt;
                    vel_vector_normal = dot(new_state.Velocity, collision_normal) * collision_normal;
                    new_state.Velocity = new_state.Velocity - vel_vector_normal;
                    new_state.Velocity = check_limits(new_state.Velocity, obj.VelocityLimits);
                    new_state.Velocity = check_limits(new_state.Velocity, obj.TotalSpeedLimit);
                    new_state.Position = obj.State.Position + new_state.Velocity * dt;
                end
            end
            
            last_collision_status = is_collision;
        end
        
        function new_state = CalcStateLagrange(obj, RotorSpeedsSquared, dt)
            % Create the new state
            new_state = state(obj.NumOfRotors);
        end
        
        function InitializeDynamicsMethod(obj)
            switch obj.DynamicsMethod
                case multirotor_dynamics_methods.NewtonEuler
                    obj.InitializeNewtonEulerMethod()
                otherwise
                    error('The specified dynamics method is not implemented yet.');
            end
        end
        
        function InitializeNewtonEulerMethod(obj)
        % Initialize the NDI method
        
            % Calculate L matrix (related to body thrust forces)
            obj.NE_L = zeros(3, obj.NumOfRotors);
            for i = 1 : obj.NumOfRotors
               obj.NE_L(:, i) = obj.Rotors{i}.GetThrustForce(1);
            end

            % Calculate G matrix (related to body reaction moments)
            NE_G = zeros(3, obj.NumOfRotors);
            for i = 1 : obj.NumOfRotors
               NE_G(:, i) = obj.Rotors{i}.GetReactionMoment(1);
            end
            
            % Calculate F matrix (related to body thrust moments)
            NE_F = zeros(3, obj.NumOfRotors);
            for i = 1 : obj.NumOfRotors
                r = obj.Rotors{i}.Position;
                F = obj.Rotors{i}.GetThrustForce(1);
                NE_F(:, i) = cross(r, F);
            end
            
            obj.NE_M = NE_F + NE_G;
            
            % Update the manipulability matrices
            obj.Ang_Acc_Manip_A = obj.I_inv * obj.GetGravityMoment(eye(3));
            obj.Ang_Acc_Manip_B = obj.I_inv * obj.NE_M;
        end
        
        function e_pos = CalcEndEffectorPosition(obj, m_pos, m_rpy_deg)
            obj.CheckEndEffector();
            RBI = physics.GetRotationMatrixDegrees(m_rpy_deg(1), m_rpy_deg(2), m_rpy_deg(3));
            e_pos = m_pos + RBI' * obj.EndEffector.EndEffectorPosition;
        end
        
        function e_vel = CalcEndEffectorVelocity(obj, m_vel, m_omega, m_rpy_deg)
        % Reference: https://www.mdpi.com/2076-3417/9/11/2230
            obj.CheckEndEffector();
            RBI = physics.GetRotationMatrixDegrees(m_rpy_deg(1), m_rpy_deg(2), m_rpy_deg(3));
            e_vel = m_vel + RBI' * cross(m_omega, obj.EndEffector.EndEffectorPosition);
        end
        
        function m_vel = CalcRobotVelocityFromEndEffector(obj, e_vel, e_omega, m_rpy_deg)
        % Reference: https://www.mdpi.com/2076-3417/9/11/2230
            obj.CheckEndEffector();
            RBI = physics.GetRotationMatrixDegrees(m_rpy_deg(1), m_rpy_deg(2), m_rpy_deg(3));
            m_vel = e_vel - RBI' * cross(e_omega, obj.EndEffector.EndEffectorPosition);
        end
        
        function R_IE = GetEndEffectorRotation(obj)
            obj.CheckEndEffector();
            RBI = obj.GetRotationMatrix();
            R_IE = RBI' * obj.EndEffector.R_BE;
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
                    FB = FB + obj.Rotors{i}.GetThrustForce(RotorSpeedsSquared(i));
                else
                    max_thrust = [0; 0; -norm(obj.Rotors{i}.GetThrustForce(obj.Rotors{i}.MaxrotorSpeedSquared))];
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
                F = obj.Rotors{i}.GetThrustForce(RotorSpeedsSquared(i));
                M = M + cross(r, F);
            end
        end
        
        function M = GetReactionMoment(obj, RotorSpeedsSquared)
            M = zeros(3, 1);
            for i = 1 : obj.NumOfRotors
               M = M + obj.Rotors{i}.GetReactionMoment(RotorSpeedsSquared(i));
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
