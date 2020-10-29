classdef control_allocation < handle
    properties
    end
    
    properties(SetAccess=protected, GetAccess=public)
        Method control_allocation_types % The control allocation method
    end
    
    properties(SetAccess=protected, GetAccess=protected)
        NDI_L                 % L matrix (related to body-fixed thrust forces)
        NDI_M                 % M matrix (related to body-fixed thrust and reaction moments)
    end

    methods
        function obj = control_allocation(multirotor)
            obj.SetMethod(multirotor, control_allocation_types.NDI);
        end
        
        function SetMethod(obj, multirotor, method)
            obj.Method = method;
            if method == control_allocation_types.NDI
                obj.InitializeNDIMethod(multirotor);
            end
        end
        
        function [rotor_speeds_squared, saturated] = CalcRotorSpeeds(obj, mult, lin_accel, ang_accel)
        % Calculate the rotor speeds from the desired linear and angular accelerations
            
            persistent lin_accel_last
            persistent ang_accel_last
            persistent rotor_speeds_squared_last
            persistent saturated_last
            if isempty(lin_accel_last)
                lin_accel_last = zeros(3, 1);
            end
            if isempty(ang_accel_last)
                ang_accel_last = zeros(3, 1);
            end
            if isempty(rotor_speeds_squared_last)
                rotor_speeds_squared_last = zeros(mult.NumOfRotors, 1);
            end
            if isempty(saturated_last)
                saturated_last = false;
            end
            
            if isequal(lin_accel, lin_accel_last) && isequal(ang_accel, ang_accel_last)
                rotor_speeds_squared = rotor_speeds_squared_last;
                saturated = saturated_last;
                return;
            end
        
            if obj.Method == control_allocation_types.NDI
                rotor_speeds_squared = obj.NDIRotorSpeeds(mult, lin_accel, ang_accel);
            end

            saturation_flag = false;
            max_rotor_speeds = cell2mat(cellfun(@(s)s.MaxSpeedSquared, mult.Rotors, 'uni', 0));
            if any(rotor_speeds_squared > max_rotor_speeds)
                %mx = max((rotor_speeds_squared - max_rotor_speeds) ./ max_rotor_speeds);
                %rotor_speeds_squared = rotor_speeds_squared - mx * max_rotor_speeds - 1e-5;
                ind = rotor_speeds_squared > max_rotor_speeds;
                rotor_speeds_squared(ind) = max_rotor_speeds(ind);
                saturation_flag = true;
            end
            min_rotor_speeds = cell2mat(cellfun(@(s)s.MinSpeedSquared, mult.Rotors, 'uni', 0));
            if any(rotor_speeds_squared < min_rotor_speeds)
                ind = rotor_speeds_squared < min_rotor_speeds;
                rotor_speeds_squared(ind) = min_rotor_speeds(ind);
                saturation_flag = true;
            end
            
            if nargin > 1
                saturated = saturation_flag;
            end
            
            lin_accel_last = lin_accel;
            ang_accel_last = ang_accel;
            saturated_last = saturated;
            rotor_speeds_squared_last = rotor_speeds_squared;
        end
    end
    
    %% Private Methods
    methods(Access=protected)
        function InitializeNDIMethod(obj, multirotor)
        % Initialize the NDI method
        
            % Calculate L matrix (related to body thrust forces)
            obj.NDI_L = zeros(3, multirotor.NumOfRotors);
            for i = 1 : multirotor.NumOfRotors
               obj.NDI_L(:, i) = multirotor.Rotors{i}.GetThrustForcePerUnitInput();
            end

            % Calculate G matrix (related to body reaction moments)
            NDI_G = zeros(3, multirotor.NumOfRotors);
            for i = 1 : multirotor.NumOfRotors
               NDI_G(:, i) = multirotor.Rotors{i}.GetReactionMomentPerUnitInput();
            end
            
            % Calculate F matrix (related to body thrust moments)
            NDI_F = zeros(3, multirotor.NumOfRotors);
            for i = 1 : multirotor.NumOfRotors
                r = multirotor.Rotors{i}.Position;
                F = multirotor.Rotors{i}.GetThrustForcePerUnitInput();
                NDI_F(:, i) = cross(r, F);
            end
            
            obj.NDI_M = NDI_F + NDI_G;
        end
        
        function rotor_speeds_squared = NDIRotorSpeeds(obj, multirotor, lin_accel, euler_accel)
        % Calculate the rotor speeds from the desired linear and angular accelerations
        % using NDI method
            
            % Create the desired output matrix y
            y = [lin_accel; euler_accel];
        
            % Get the rotation matrix
            RBI = multirotor.GetRotationMatrix();
            
            % Calculate eta_dot
            phi = deg2rad(multirotor.State.RPY(1));
            theta = deg2rad(multirotor.State.RPY(2));
            phi_dot = deg2rad(multirotor.State.EulerRate(1));
            theta_dot = deg2rad(multirotor.State.EulerRate(2));
            eta_dot = calc_eta_dot(phi, theta, phi_dot, theta_dot);
            
            % Calculate eta
            eta = [1,   sin(phi)*tan(theta), cos(phi)*tan(theta);
                   0, cos(phi), -sin(phi);
                   0, sin(phi) / cos(theta), cos(phi) / cos(theta)];

            % Calculate the A matrix in y = A + Bu
            NDI_M_Grav = zeros(3, 1);
            for i = 1 : multirotor.NumOfRotors
                r = multirotor.Rotors{i}.Position;
                G_motor = multirotor.Rotors{i}.MotorMass * physics.Gravity;
                G_motorB = RBI * G_motor;
                G_arm = multirotor.Rotors{i}.ArmMass * physics.Gravity;
                G_armB = RBI * G_arm;
                NDI_M_Grav = NDI_M_Grav + cross(r, G_motorB) + cross(r/2, G_armB);
            end
            if multirotor.HasEndEffector()
                r = multirotor.EndEffector.EndEffectorPosition;
                G_eeI = multirotor.EndEffector.EndEffectorMass * physics.Gravity;
                G_eeB = RBI * G_eeI;
                G_armI = multirotor.EndEffector.ArmMass * physics.Gravity;
                G_armB = RBI * G_armI;
                NDI_M_Grav = NDI_M_Grav + cross(r, G_eeB) + cross(r/2, G_armB);
            end
            
            A_moment = eta_dot * multirotor.State.Omega + eta * multirotor.I_inv * ...
                (NDI_M_Grav - cross(multirotor.State.Omega, multirotor.I * multirotor.State.Omega));
            A = [physics.Gravity; A_moment];
            
            % Calculate the B matrix
            B_force = RBI' * obj.NDI_L / multirotor.TotalMass;
            B_moment = eta * multirotor.I_inv * obj.NDI_M;
            B = [B_force; B_moment];
            
            % Calculate the rotor speeds
            rotor_speeds_squared = pinv(B) * (y - A); 
        end
    end
end

%% Other functions
function eta_dot = calc_eta_dot(phi, theta, phi_dot, theta_dot)
    eta_dot_11 = 0;
    eta_dot_12 = sin(phi)*(tan(theta)^2 + 1)*theta_dot + cos(phi)*tan(theta)*phi_dot;
    eta_dot_13 = cos(phi)*(tan(theta)^2 + 1)*theta_dot - sin(phi)*tan(theta)*phi_dot;

    eta_dot_21 = 0;
    eta_dot_22 = -phi_dot*sin(phi);
    eta_dot_23 = -phi_dot*cos(phi);

    eta_dot_31 = 0;
    eta_dot_32 = (cos(phi)*phi_dot)/cos(theta) + (sin(phi)*sin(theta)*theta_dot)/cos(theta)^2;
    eta_dot_33 = (cos(phi)*sin(theta)*theta_dot)/cos(theta)^2 - (sin(phi)*phi_dot)/cos(theta);

    eta_dot = [eta_dot_11 eta_dot_12 eta_dot_13;
               eta_dot_21 eta_dot_22 eta_dot_23;
               eta_dot_31 eta_dot_32 eta_dot_33];
end
