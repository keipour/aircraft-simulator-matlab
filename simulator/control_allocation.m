classdef control_allocation < handle
    properties
        Method control_allocation_types % The control allocation method
        Multirotor multirotor           % The multirotor object
    end
    
    properties(SetAccess=protected, GetAccess=public)
    end
    
    properties(SetAccess=protected, GetAccess=protected)
       NDI_L        % L matrix (related to body-fixed thrust forces)
       NDI_M        % M matrix (related to body-fixed thrust and reaction moments)
       NDI_M_Grav   % M_Grav vector (related to the body-fixed gravity moments)
    end

    methods
        function obj = control_allocation(multirotor)
            obj.Multirotor = multirotor;
            obj.Method = control_allocation_types.NDI;
        end
        
        function set.Method(obj, value)
            obj.Method = value;
            if value == control_allocation_types.NDI
                obj.InitializeNDIMethod();
            end
        end
        
        function rotor_speeds_squared = CalcRotorSpeeds(obj, state, lin_accel, ang_accel)
        % Calculate the rotor speeds from the desired linear and angular accelerations
            
            % Create the desired output matrix y
            y = [lin_accel; ang_accel];
        
            % Calculate eta_dot
            phi = deg2rad(state.RPY(1));
            theta = deg2rad(state.RPY(2));
            phi_dot = deg2rad(state.EulerDerivative(1));
            theta_dot = deg2rad(state.EulerDerivative(2));
            eta_dot = CalcEtaDot(phi, theta, phi_dot, theta_dot);
            
            % Calculate eta
            eta = [1,   sin(phi)*tan(theta), cos(phi)*tan(theta);
                   0, cos(phi), -sin(phi);
                   0, sin(phi) / cos(theta), cos(phi) / cos(theta)];

            % Calculate the B matrix in y = A + Bu
            B_moment = eta_dot * state.Omega + eta * obj.Multirotor.I_inv * ...
                (obj.NDI_M_Grav - cross(state.Omega, obj.Multirotor.I * state.Omega));
            B = [obj.Multirotor.Gravity; B_moment];
            
            % Calculate the A matrix
            %A_force = 
        end
    end
    
    %% Private Methods
    methods(Access=protected)
        function InitializeNDIMethod(obj)
        % Initialize the NDI method
        
            % Calculate L matrix (related to body thrust forces)
            obj.NDI_L = zeros(3, obj.Multirotor.NumOfRotors);
            for i = 1 : obj.Multirotor.NumOfRotors
               obj.NDI_L(:, i) = rotor.GetThrustForce(obj.Multirotor.Rotors{i}, 1);
            end

            % Calculate G matrix (related to body reaction moments)
            NDI_G = zeros(3, obj.Multirotor.NumOfRotors);
            for i = 1 : obj.Multirotor.NumOfRotors
               NDI_G(:, i) = rotor.GetReactionMoment(obj.Multirotor.Rotors{i}, 1);
            end
            
            % Calculate F matrix (related to body thrust moments)
            NDI_F = zeros(3, obj.Multirotor.NumOfRotors);
            for i = 1 : obj.Multirotor.NumOfRotors
                r = obj.Multirotor.Rotors{i}.Position;
                F = rotor.GetThrustForce(obj.Multirotor.Rotors{i}, 1);
                NDI_F(:, i) = cross(r, F);
            end
            
            obj.NDI_M = NDI_F + NDI_G;

            obj.NDI_M_Grav = zeros(3, 1); % WRONGGGGG
            for i = 1 : obj.Multirotor.NumOfRotors
                r = obj.Multirotor.Rotors{i}.Position;
                G_motor = obj.Multirotor.Rotors{i}.MotorMass * obj.Multirotor.Gravity;
                G_motorB = obj.Multirotor.Rotors{i}.R * G_motor;
                G_arm = obj.Multirotor.Rotors{i}.ArmMass * obj.Multirotor.Gravity;
                G_armB = obj.Multirotor.Rotors{i}.R * G_arm;
                obj.NDI_M_Grav = obj.NDI_M_Grav + cross(r, G_motorB) + cross(r/2, G_armB);
            end
        end
        
        function eta_dot = CalcEtaDot(phi, theta, phi_dot, theta_dot)
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
    end
end
