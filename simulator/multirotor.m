%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% Copyright (c) 2020 Carnegie Mellon University
% This tool has been developed for educational purposes only as a 
% control tutorial in Air Lab Summer School 2020 (https://theairlab.org). 
% Azarakhsh Keipour (keipour [at] cmu.edu)
% Please contact us for any questions or issues.
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

classdef multirotor < handle
    %PLANT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Fixed Properties
        Rotors
        Mass = 3.0;                 % in Kg
        I                           % Inertia
        PayloadRadius = 0.15;       % in meters
        
        TotalSpeedLimit = 20;                       % in m/s
        VelocityLimits = [10; 10; 8];               % in m/s
        EulerRateLimits = [70; 70; 30];       % in deg/s
    end

    properties(SetAccess=protected, GetAccess=public)
        NumOfRotors                 % Number of rotors
        InitialState                % Initial state
        State                       % The current state
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
        
        function SetInitialState(obj, pos, vel, rpy, omega)
            obj.InitialState.Position = pos;
            obj.InitialState.Velocity = vel;
            obj.InitialState.RPY = rpy;
            obj.InitialState.Omega = omega;
            
            obj.State.Position = pos;
            obj.State.Velocity = vel;
            obj.State.RPY = rpy;
            obj.State.Omega = omega;
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
        
        function UpdateState(obj, RotorSpeedsSquared, dt)
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
            obj.State.EulerRate = phi_dot;
            obj.State.AngularAcceleration = omega_dot;
            
            for i = 1 : obj.NumOfRotors
                [rs, sat] = rotor.LimitRotorSpeed(obj.Rotors{i}, RotorSpeedsSquared(i));
                obj.State.RotorSpeeds(i) = sqrt(rs);
                obj.State.RotorsSaturated = obj.State.RotorsSaturated || sat;
            end
        end
        
        function set.I(obj, value)
            obj.I = value;
            obj.UpdateI_inv();
        end
        
        function UpdateStructure(obj)
            obj.I = obj.EstimateInertia();
            obj.UpdateNumOfRotors();
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
        end
        
        function Visualize(obj)
            graphics.VisualizeMultirotor(obj);
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
        
        function UpdateNumOfRotors(obj)
            obj.NumOfRotors = length(obj.Rotors);
        end
        
        function UpdateI_inv(obj)
            obj.I_inv = pinv(obj.I);
        end

        function F = GetGravityForce(obj)
            F = physics.Gravity * obj.Mass;
        end
        
        function F = GetThrustForce(obj, Rot_IB, RotorSpeedsSquared)
            FB = zeros(3, 1);
            for i = 1 : obj.NumOfRotors
               FB = FB + rotor.GetThrustForce(obj.Rotors{i}, RotorSpeedsSquared(i));
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
            p_dotdot = force / obj.Mass;
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
            phi_dot = wrapTo180(rad2deg(eta * obj.State.Omega));
            phi_dot = check_limits(phi_dot, obj.EulerRateLimits);
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
