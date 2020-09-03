classdef physics
    
    properties(Constant)
        Gravity = [0; 0; 9.80665];  % in m/s^2
    end
    
    methods(Static)
        % Estimate the inertia tensor of the multirotor
        function inertia_tensor = EstimateInertia(multirotor)

            % Initialization
            num_rotors = multirotor.NumOfRotors;
            arm_lengths = cell2mat(cellfun(@(s)s.ArmLength', multirotor.Rotors, 'uni', 0));
            arm_angles = cell2mat(cellfun(@(s)s.ArmAngle', multirotor.Rotors, 'uni', 0)) * pi / 180;
            phi_dihs = cell2mat(cellfun(@(s)s.DihedralAngle', multirotor.Rotors, 'uni', 0)) * pi / 180;
            mass_arms = cell2mat(cellfun(@(s)s.ArmMass', multirotor.Rotors, 'uni', 0));
            mass_motors = cell2mat(cellfun(@(s)s.MotorMass', multirotor.Rotors, 'uni', 0));
            mass_payload = multirotor.Mass - sum(mass_arms) - sum(mass_motors);
            payload_radius = multirotor.PayloadRadius;

            % Calculate the rotor coordinates (which also serve as the end points for arms)
            X_rotors = arm_lengths .* cos(phi_dihs) .* cos(arm_angles);
            Y_rotors = arm_lengths .* cos(phi_dihs) .* sin(arm_angles);
            Z_rotors = arm_lengths .* sin(-phi_dihs);

            % Calculate the payload tensor (the main mass around the center assuming that it's a perfect sphere)
            payload_tensor = solid_sphere_inertia(payload_radius, mass_payload);

            % Calculate the rotors tensor (sum of all the rotors as point masses)
            rotor_tensor = cell(num_rotors, 1);
            for i = 1 : num_rotors
                rotor_tensor{i} = point_mass_inertia(mass_motors(i), X_rotors(i), Y_rotors(i), Z_rotors(i));
            end

            % Calculate the arm tensor (sum of all the arms as rods connecting center to rotors)
            arm_tensor = cell(num_rotors, 1);
            for i = 1 : num_rotors
                arm_tensor{i} = support_files.rod_inertia(mass_arms(i), 0, 0, 0, X_rotors(i), Y_rotors(i), Z_rotors(i));
            end
            
            % Calculate the overall tensor as the sum of all the tensors
            inertia_tensor = payload_tensor;
            for i = 1 : num_rotors
                inertia_tensor = inertia_tensor + rotor_tensor{i} + arm_tensor{i};
            end
            
            % Calculate and add the manipulator arm tensor
            % it is modeled as a rod with a point mass at the end-effector
            if ~multirotor.HasEndEffector()
                return;
            end
            mass_ee_arm = multirotor.EndEffector.ArmMass;
            mass_ee_end = multirotor.EndEffector.EndEffectorMass;
            base_pos = multirotor.EndEffector.BasePosition;
            ee_pos = multirotor.EndEffector.EndEffectorPosition;
            arm_rod_tensor = support_files.rod_inertia(mass_ee_arm, base_pos(1), ...
                base_pos(2), base_pos(3), ee_pos(1), ee_pos(2), ee_pos(3));
            ee_tensor = point_mass_inertia(mass_ee_end, ee_pos(1), ee_pos(2), ee_pos(3));

            inertia_tensor = inertia_tensor + arm_rod_tensor + ee_tensor;
        end
        
        function RNI = GetRotationMatrixRadians(roll, pitch, yaw)
            angles = [yaw pitch roll];

            RNI = zeros(3, 3);
            cang = cos(angles);
            sang = sin(angles);

            %     [          cy*cz,          cy*sz,            -sy]
            %     [ sy*sx*cz-sz*cx, sy*sx*sz+cz*cx,          cy*sx]
            %     [ sy*cx*cz+sz*sx, sy*cx*sz-cz*sx,          cy*cx]

            RNI(1,1) = cang(2).*cang(1);
            RNI(1,2) = cang(2).*sang(1);
            RNI(1,3) = -sang(2);
            RNI(2,1) = sang(3).*sang(2).*cang(1) - cang(3).*sang(1);
            RNI(2,2) = sang(3).*sang(2).*sang(1) + cang(3).*cang(1);
            RNI(2,3) = sang(3).*cang(2);
            RNI(3,1) = cang(3).*sang(2).*cang(1) + sang(3).*sang(1);
            RNI(3,2) = cang(3).*sang(2).*sang(1) - sang(3).*cang(1);
            RNI(3,3) = cang(3).*cang(2);
        end

        function RNI = GetRotationMatrixDegrees(roll, pitch, yaw)
            r = deg2rad(roll);
            p = deg2rad(pitch);
            y = deg2rad(yaw);
            RNI = physics.GetRotationMatrixRadians(r, p, y);
        end

        function flag = CheckAllCollisions(set1, set2)
        % Check collision between two different sets of collision geometries

            flag = false;
            for i = 1 : length(set1)
                for j = 1 : length(set2)
                    if physics.CheckCollision(set1{i}, set2{j})
                        flag = true;
                        return;
                    end
                end 
            end
        end

        function [collisionStatus, separationDist, witnessPts] = CheckCollision(geom1, geom2)
            [collisionStatus, separationDist, witnessPts] = ...
                    robotics.core.internal.intersect(geom1.GeometryInternal, geom1.Position, geom1.Quaternion,...
                                                     geom2.GeometryInternal, geom2.Position, geom2.Quaternion, 1);
            if collisionStatus
                separationDist = nan;
                witnessPts = nan(3,2);
            end
        end
        
    end
end

%% Calculate the intertia tensor of a solid sphere around the center
function inertia_tensor = solid_sphere_inertia(radius, mass)
    inertia_tensor = (2/5 * mass * radius^2) * eye(3);
end

%% Calculate the intertia tensor of a rod from point 1 to 2 around the center
% function inertia_tensor = rod_inertia(mass, x1, y1, z1, x2, y2, z2)
%     N = 1e5; % Numerical calculation fidelity
% 
%     x_lin = linspace(x1, x2, N);
%     y_lin = linspace(y1, y2, N);
%     z_lin = linspace(z1, z2, N);
%     dm = mass / length(x_lin);
% 
%     % Calculate the tensor by summing the point masses
%     inertia_tensor  = zeros(3);
%     for i = 1 : length(x_lin)
%         inertia_tensor = inertia_tensor + point_mass_inertia(dm, x_lin(i), y_lin(i), z_lin(i));
%     end
% end

%% Calculate the intertia tensor of a point mass at x, y, z around the center
function inertia_tensor = point_mass_inertia(mass, x, y, z)
    Ixx = mass * (y^2 + z^2);
    Iyy = mass * (x^2 + z^2);
    Izz = mass * (x^2 + y^2);
    Ixy = -mass * x * y;
    Ixz = -mass * x * z;
    Iyz = -mass * y * z;
    
    inertia_tensor = [  Ixx     Ixy     Ixz
                        Ixy     Iyy     Iyz
                        Ixz     Iyz     Izz  ];
end
