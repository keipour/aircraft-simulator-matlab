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
        CollisionModel
    end
    
    properties(SetAccess=protected, GetAccess=protected)
        HasArm = false;
        TransformedCollisionModels
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
            
            if obj.HasArm
                obj.State.EndEffectorPosition = obj.CalcEndEffectorPosition(obj.State.Position, obj.State.RPY);
                obj.State.EndEffectorVelocity = obj.CalcEndEffectorVelocity(obj.State.Velocity, obj.State.Omega, obj.State.RPY);
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
                obj.Rotors{i} = rotor.SetInwardAngle(obj.Rotors{i}, RotorInwardAngles(i));
                obj.Rotors{i} = rotor.SetSidewardAngle(obj.Rotors{i}, RotorSidewardAngles(i));
                obj.Rotors{i} = rotor.SetDihedralAngle(obj.Rotors{i}, RotorDihedralAngles(i));
            end
            
            % Update the structure
            obj.UpdateStructure();
        end
        
        function UpdateState(obj, new_state)
            obj.State = new_state;
        end
        
        function [wrench] = CalcForceWrench(obj, RotorSpeedsSquared)

            % Calculate the current rotation matrix
            RBI = obj.GetRotationMatrix();

            % Calculate the total force
            force = obj.GetGravityForce() + ...
                obj.GetThrustForce(RBI', RotorSpeedsSquared);

            % Calculate the total moment
            moment = obj.GetGravityMoment(RBI) + ...
                obj.GetThrustMoment(RotorSpeedsSquared) + ...
                obj.GetReactionMoment(RotorSpeedsSquared);
            
            wrench = [moment; force];
        end
        
        function new_state = CalcNextState(obj, wrench, tf_sensor_wrench, ...
                RotorSpeedsSquared, dt, is_collision, collision_normal)

            force_moment_ext_B = zeros(3, 1);
            force_ext_I = zeros(3, 1);
            if obj.HasArm
                force_ext_B = obj.EndEffector.R_BE * tf_sensor_wrench(4:6);
                moment_ext_B = obj.EndEffector.R_BE * tf_sensor_wrench(1:3);
                force_moment_ext_B = cross(obj.EndEffector.EndEffectorPosition, force_ext_B);
                force_ext_I = obj.GetRotationMatrix()' * force_ext_B;
            end
            
            % Calculate the time step
            new_state = obj.CalcStateNewtonEuler(wrench, [zeros(3, 1); force_ext_I], ...
                dt, is_collision, collision_normal);
            
            % Save the force and moment data
            new_state.Force = wrench(4:6);
            new_state.Moment = wrench(1:3);
            new_state.ForceSensor = tf_sensor_wrench(4:6);
            new_state.MomentSensor = tf_sensor_wrench(1:3);
            new_state.InCollision = is_collision;
            
            for i = 1 : obj.NumOfRotors
                [rs, sat] = rotor.LimitRotorSpeed(obj.Rotors{i}, RotorSpeedsSquared(i));
                new_state.RotorSpeeds(i) = sqrt(rs);
                new_state.RotorsSaturated = new_state.RotorsSaturated || sat;
            end
            
            if obj.HasArm
                new_state.EndEffectorPosition = obj.CalcEndEffectorPosition(new_state.Position, new_state.RPY);
                new_state.EndEffectorVelocity = obj.CalcEndEffectorVelocity(new_state.Velocity, new_state.Omega, new_state.RPY);
            end
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
            obj.TotalMass = obj.Mass;
            obj.CollisionModel = obj.CalculateCollisionModel();
            mult_cm = collisionBox(obj.CollisionModel.X, ...
                obj.CollisionModel.Y, obj.CollisionModel.Z);

            if obj.HasArm
                obj.TotalMass = obj.TotalMass + obj.EndEffector.TotalMass;
                arm_cm = collisionBox(obj.EndEffector.CollisionModel.X, ...
                    obj.EndEffector.CollisionModel.Y,...
                    obj.EndEffector.CollisionModel.Z);
                obj.TransformedCollisionModels = {mult_cm, arm_cm};
            else
                obj.TransformedCollisionModels = {mult_cm};
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
            obj.HasArm = mult.HasEndEffector();
            obj.TotalSpeedLimit = mult.TotalSpeedLimit;
            obj.VelocityLimits = mult.VelocityLimits;
            obj.OmegaLimits = mult.OmegaLimits;
            obj.TotalMass = mult.TotalMass;
            obj.CollisionModel = collisionBox(mult.CollisionModel.X, ...
                mult.CollisionModel.Y, mult.CollisionModel.Z);
            obj.CollisionModel.Pose = mult.CollisionModel.Pose;

            mult_cm = collisionBox(mult.CollisionModel.X, ...
                mult.CollisionModel.Y, mult.CollisionModel.Z);
            if obj.HasArm
                arm_cm = collisionBox(mult.EndEffector.CollisionModel.X, ...
                    mult.EndEffector.CollisionModel.Y,...
                    mult.EndEffector.CollisionModel.Z);
                obj.TransformedCollisionModels = {mult_cm, arm_cm};

                obj.EndEffector = arm;
                obj.EndEffector.CopyFrom(mult.EndEffector);
            else
                obj.TransformedCollisionModels = {mult_cm};
            end
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
            
            cm = collisionBox(x(2) - x(1), y(2) - y(1), z(2) - z(1));
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
        
        function new_state = CalcStateNewtonEuler(obj, wrench, ext_wrench, ...
                dt, is_collision, collision_normal)
            
            % Create the new state
            new_state = state.Create(obj.NumOfRotors);
            
            % Get the total force
            force = wrench(4:6) + ext_wrench(4:6);
            moment = wrench(1:3) + ext_wrench(1:3);
            
            % Calculate the equations of motion
            p_dotdot = obj.GetLinearAcceleration(force);
            omega_dot = obj.GetAngularAcceleration(moment);
            phi_dot = obj.GetEulerRate();
            
            % I assume approximately constant acceleration to update these
            % first before other variables
            new_state.Acceleration = p_dotdot;
            new_state.AngularAcceleration = omega_dot;

            % If there is a collision
            if is_collision
                % I assume no bouncing due to the impact and no impulse
                % TODO: Model based on https://www.sciencedirect.com/science/article/abs/pii/S0094114X02000459
                % We assume that the collision happened at the end effector
                % if it has end effector
                if obj.HasArm
                    new_state.EndEffectorVelocity = obj.State.EndEffectorVelocity + new_state.Acceleration * dt;
                    vel_vector_normal = dot(new_state.EndEffectorVelocity, collision_normal) * collision_normal;
                    new_state.EndEffectorVelocity = new_state.EndEffectorVelocity - vel_vector_normal;
                    new_state.EndEffectorPosition = obj.State.EndEffectorPosition + new_state.EndEffectorVelocity * dt;
                    RBI = obj.GetRotationMatrix();
                    new_state.Position = new_state.EndEffectorPosition - RBI' * obj.EndEffector.EndEffectorPosition;
                    new_state.Velocity = (new_state.Position - obj.State.Position) / dt;
                else
                    new_state.Velocity = obj.State.Velocity + new_state.Acceleration * dt;
                    vel_vector_normal = dot(new_state.Velocity, collision_normal) * collision_normal;
                    new_state.Velocity = new_state.Velocity - vel_vector_normal;
                    new_state.Velocity = check_limits(new_state.Velocity, obj.VelocityLimits);
                    new_state.Velocity = check_limits(new_state.Velocity, obj.TotalSpeedLimit);
                    new_state.Position = obj.State.Position + new_state.Velocity * dt;
                end
            else
                new_state.Position = obj.State.Position + 0.5 * new_state.Acceleration * dt * dt + ...
                    obj.State.Velocity * dt;

                new_state.Velocity = obj.State.Velocity + new_state.Acceleration * dt;
                new_state.Velocity = check_limits(new_state.Velocity, obj.VelocityLimits);
                new_state.Velocity = check_limits(new_state.Velocity, obj.TotalSpeedLimit);
            end
            
            % Update the rest of the state
            new_state.RPY = wrapTo180(obj.State.RPY + obj.State.EulerRate * dt);
            new_state.Omega = obj.State.Omega + new_state.AngularAcceleration * dt;
            new_state.Omega = check_limits(new_state.Omega, obj.OmegaLimits);

            new_state.EulerRate = phi_dot;
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
