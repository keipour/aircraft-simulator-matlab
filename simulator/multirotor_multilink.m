classdef multirotor_multilink < handle
    properties
        MainLink multirotor
        ConnectedLinks cell = {};
        Rods cell = {};        % Rods in the system (except for exnd-effector)
    end

    properties (SetAccess=protected, GetAccess=public)
        % Add your properties that can only be set internally
    end
    
    properties (SetAccess=protected, GetAccess=protected)
        % Add your properties that can only be seen internally
    end
    
    %% Public methods
    methods
        function obj = multirotor_multilink(main_link)
            obj.MainLink = main_link;
        end
        
        function AddRod(obj, start_point, end_point)
            num_of_rods = length(obj.Rods) + 1;
            obj.Rods{num_of_rods} = {};
            obj.Rods{num_of_rods}.Start = start_point;
            obj.Rods{num_of_rods}.End = end_point;
        end
        
        function wrench = CalcGeneratedWrench(obj, rotor_speeds_squared)
            wrench = obj.MainLink.CalcGeneratedWrench(rotor_speeds_squared);
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
            
            % Calculate the tilt
            RIB = obj.GetRotationMatrix()';
            z_axis = RIB(:, 3);
            z_axis(abs(z_axis) < 1e-5) = 0;
            new_state.TiltAngle =  acosd(z_axis(3));
            new_state.TiltDirection = atan2d(-z_axis(2), -z_axis(1));
            
            for i = 1 : obj.NumOfRotors
                [rs, sat] = obj.Rotors{i}.LimitRotorSpeed(RotorSpeedsSquared(i));
                new_state.RotorSpeeds(i) = sqrt(rs);
                new_state.RotorsSaturated = new_state.RotorsSaturated || sat;
                new_state.RotorInwardAngles(i) = obj.Rotors{i}.InwardAngle;
                new_state.RotorSidewardAngles(i) = obj.Rotors{i}.SidewardAngle;
            end
            
            for i = 1 : obj.NumOfServos
                new_state.ServoAngles(i) = obj.Servos{i}.CurrentAngle;
            end
        end
        
        function accel = CalculateAccelerationManipulability(obj, wind_force, rotor_speeds_squared, get_maximum)
            if nargin < 4
                get_maximum = false;
            end
            
            persistent F RPY
            if isempty(F) || isempty(RPY)
                RPY = obj.State.RPY;
                F = (obj.GetRotationMatrix()' * obj.NE_L) / obj.TotalMass;
            end

            if get_maximum
                accel = (obj.GetGravityForce() + ...
                    obj.GetThrustForce(eye(3), rotor_speeds_squared, get_maximum)) / obj.TotalMass;
            else
                if ~isequal(RPY, obj.State.RPY)
                    RPY = obj.State.RPY;
                    F = (obj.GetRotationMatrix()' * obj.NE_L) / obj.TotalMass;
                end
 
                accel = physics.Gravity + wind_force / obj.TotalMass + F * rotor_speeds_squared;
            end
        end
        
        function omega_dot = CalculateAngularAccelerationManipulability(obj, rotor_speeds_squared)
            
            persistent M1 M2 RPY Omega
            if isempty(M1) || isempty(M2) || isempty(RPY) || isempty(Omega)
                RPY = obj.State.RPY;
                Omega = obj.State.Omega;
                M1 = obj.I_inv * (obj.GetGravityMoment(obj.GetRotationMatrix()) - cross(obj.State.Omega, obj.I * obj.State.Omega));
                M2 = obj.I_inv * obj.NE_M;
            end
            
            if ~isequal(RPY, obj.State.RPY) || ~isequal(Omega, obj.State.Omega)
                RPY = obj.State.RPY;
                Omega = obj.State.Omega;
                M1 = obj.I_inv * (obj.GetGravityMoment(obj.GetRotationMatrix()) - cross(obj.State.Omega, obj.I * obj.State.Omega));
            end
            
            omega_dot = M1 + M2 * rotor_speeds_squared;
        end
        
        function set.I(obj, value)
            obj.I = value;
            obj.UpdateI_inv();
        end
        
        function UpdateStructure(obj)
            obj.I = obj.EstimateInertia();
            obj.UpdateNumOfRotors();
            obj.UpdateNumOfServos();
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
            obj.Servos = {};
            for i = 1 : mult.NumOfServos
                obj.AddServo(mult.Servos{i}.RotorNumbers, mult.Servos{i}.Axes, mult.Servos{i}.CurrentAngle);
            end
            obj.Rods = {};
            for i = 1 : length(mult.Rods)
                obj.Rods{i} = mult.Rods{i};
            end
            obj.Mass = mult.Mass;
            obj.I = mult.I;
            obj.PayloadRadius = mult.PayloadRadius;
            obj.NumOfRotors = mult.NumOfRotors;
            obj.NumOfServos = mult.NumOfServos;
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
        
        function [H_m, H_r] = Visualize(obj, draw_collision_model)
            if nargin < 2
                draw_collision_model = false;
            end
            [H_m, H_r] = graphics.VisualizeMultirotor(obj, false, draw_collision_model);
        end
        
        function H = VisualizeAxes(obj)
            [H, ~] = graphics.VisualizeMultirotor(obj, true);
        end
        
        function result = AnalyzeDynamicManipulability(obj, wind_force)
            if nargin < 2
                wind_force = zeros(3, 1);
            end
            result = analysis.AnalyzeDynamicManipulability(obj, wind_force);
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
    
end
