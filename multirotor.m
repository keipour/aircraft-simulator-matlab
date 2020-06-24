classdef multirotor
    %PLANT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Fixed Properties
        Rotors
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
            obj.Rotors = cell(obj.NumOfRotors, 1);
            
            for i = 1 : obj.NumOfRotors
                obj.Rotors{i} = rotor_create();
                obj.Rotors{i} = rotor_set_arm_angle(obj.Rotors{i}, ArmAngles(i));
                obj.Rotors{i}.RotationDirection = RotationDirections(i);
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
                obj.Rotors{i} = rotor_set_inward_angle(obj.Rotors{i}, RotorInwardAngles(i));
                obj.Rotors{i} = rotor_set_sideward_angle(obj.Rotors{i}, RotorSidewardAngles(i));
                obj.Rotors{i} = rotor_set_dihedral_angle(obj.Rotors{i}, RotorDihedralAngles(i));
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
               F = F + rotor_get_thrust_force(obj.Rotors{i}, RotorSpeedsSquared(i));
            end
        end
        
        function M = GetGravityMoment(obj)
            M = zeros(3, 1);
            for i = 1 : obj.NumOfRotors
                r = obj.Rotors{i}.Position;
                G_motor = obj.Rotors{i}.MotorMass * obj.Gravity;
                G_motorB = obj.Rotors{i}.R * G_motor;
                G_arm = obj.Rotors{i}.ArmMass * obj.Gravity;
                G_armB = obj.Rotors{i}.R * G_arm;
                M = M + cross(r, G_motorB) + cross(r/2, G_armB);
            end
        end
        
        function M = GetThrustMoment(obj, RotorSpeedsSquared)
            M = zeros(3, 1);
            for i = 1 : obj.NumOfRotors
                r = obj.Rotors{i}.Position;
                F = rotor_get_thrust_force(obj.Rotors{i}, RotorSpeedsSquared(i));
                M = M + cross(r, F);
            end
        end
        
        function M = GetReactionMoment(obj, RotorSpeedsSquared)
            M = zeros(3, 1);
            for i = 1 : obj.NumOfRotors
               M = M + rotor_get_reaction_moment(obj.Rotors{i}, RotorSpeedsSquared(i));
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

%%

function rotor = rotor_create()
    rotor.InwardAngle = 0;        % in degrees
    rotor.SidewardAngle = 0;      % in degrees
    rotor.DihedralAngle = 0;      % in degrees
    rotor.ArmAngle = 0;           % in degrees
    rotor.ArmLength = 0.4;        % in meters
    rotor.TimeConstant = 0.01;    % for motor in secs
    rotor.ConstantGain = 1;       % for motor
    rotor.RPMLimit = 3000;        % for motor
    rotor.ThrustConstant = 1.08105e-4;
    rotor.TorqueConstant = 0.05;
    rotor.MotorMass = 0.10;       % in Kilograms
    rotor.ArmMass = 0.150;        % in Kilograms
    rotor.rotorationDirection = 1;  % -1 for CW, 1 for CCW around Z
                                  % Remember that Z is downward
                               
    rotor.R = eye(3);             % rotoration matrix RRB
    rotor.Position = zeros(3, 1); % Position of the rotor in B
    rotor.MaxrotorSpeedSquared = 0;
    
    rotor = rotor_update_structure(rotor);
end

function rotor = rotor_update_structure(rotor)
    rotor.R = rotor_calc_rotoration_matrix(rotor);
    rotor.MaxrotorSpeedSquared = (rotor.RPMLimit / 30 * pi).^2;
    rotor.Position = rotor_get_position(rotor);
end

function R = rotor_calc_rotoration_matrix(rotor)
    if isempty(rotor.ArmAngle)
        R = eye(3);
        return;
    end

    mu = deg2rad(rotor.ArmAngle);            
    phix = deg2rad(rotor.InwardAngle);
    phiy = deg2rad(rotor.SidewardAngle);

    rotorZB1 = [0, 1, 0; 
              -1, 0, 0; 
              0, 0, 1];

    rotorZB2 = [cos(mu), sin(mu), 0; 
              -sin(mu), cos(mu), 0;
              0, 0, 1];

    rotorZB = rotorZB2 * rotorZB1;

    rotorXp = [1, 0, 0;
             0, cos(phix), sin(phix);
             0, -sin(phix), cos(phix)];

    rotorYpp = [cos(phiy), 0, -sin(phiy);
              0, 1, 0;
              sin(phiy), 0, cos(phiy)];

    R = rotorYpp * rotorXp * rotorZB;
end

function r = rotor_get_position(rotor)
    rx = rotor.ArmLength * cosd(rotor.DihedralAngle) * cosd(rotor.ArmAngle);
    ry = rotor.ArmLength * cosd(rotor.DihedralAngle) * sind(rotor.ArmAngle);
    rz = -rotor.ArmLength * sind(rotor.DihedralAngle);
    r = [rx; ry; rz];
end
        
function rotor = rotor_set_inward_angle(rotor, value)
    rotor.InwardAngle = value;
    rotor = rotor_update_structure(rotor);
end

function rotor = rotor_set_dihedral_angle(rotor, value)
    rotor.DihedralAngle = value;
    rotor = rotor_update_structure(rotor);
end

function rotor = rotor_set_arm_length(rotor, value)
    rotor.ArmLength = value;
    rotor = rotor_update_structure(rotor);
end
        
function rotor = rotor_set_sideward_angle(rotor, value)
    rotor.SidewardAngle = value;
    rotor = rotor_update_structure(rotor);
end

function rotor = rotor_set_arm_angle(rotor, value)
    rotor.ArmAngle = value;
    rotor = rotor_update_structure(rotor);
end

function rotor = rotor_set_rpm_limit(rotor, value)
    rotor.RPMLimit = value;
    rotor = rotor_update_structure(rotor);
end
        
function F = rotor_get_thrust_force(rotor, rotor_speed_squared)
    rotor_speed_squared = min(rotor_speed_squared, rotor.MaxrotorSpeedSquared);
    F = rotor.R' * [0; 0; -rotor.ThrustConstant * rotor_speed_squared];
end

function M = rotor_get_reaction_moment(rotor, rotor_speed_squared)
    rotor_speed_squared = min(rotor_speed_squared, rotor.MaxrotorSpeedSquared);
    M = rotor.R' * [0; 0; rotor.rotorationDirection * rotor.TorqueConstant ...
        * rotor_speed_squared];
end
