classdef physics
    
    properties(Constant)
        Gravity = [0; 0; 9.80665];  % in m/s^2
        AirDensity = 1.229          % in kg/m^3
    end
    
    methods(Static)
        % Estimate the inertia tensor of the multirotor
        function inertia_tensor = EstimateInertia(multirotor)
            
            % Note that it does not consider the weights of the rods right
            % now and considers only arms from motors to center.
            % TODO: needs to consider the rod weights.

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

        function RPY = GetRPYRadians(RNI)
            %     [          cy*cz,          cy*sz,            -sy]
            %     [ sy*sx*cz-sz*cx, sy*sx*sz+cz*cx,          cy*sx]
            %     [ sy*cx*cz+sz*sx, sy*cx*sz-cz*sx,          cy*cx]

            [yaw, pitch, roll] = threeaxisrot( RNI(1,2,:), RNI(1,1,:), -RNI(1,3,:), ...
                RNI(2,3,:), RNI(3,3,:));
            RPY = [roll; pitch; yaw];
        end
        
        function RPY = GetRPYDegrees(RNI)
            RPY = rad2deg(physics.GetRPYRadians(RNI));
        end
        
        function [set1_ind, set2_ind] = CheckAllCollisions(set1, set2)
        % Check collision between two different sets of collision geometries

            set1_ind = 0;
            set2_ind = 0;
            for i = 1 : length(set1)
                for j = 1 : length(set2)
                    if physics.CheckCollision(set1{i}, set2{j})
                        set1_ind = i;
                        set2_ind = j;
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
        
        function quat = RotationMatrixToQuaternion(R)
            if R(3, 3) < 0
                if R(1, 1) > R(2, 2)
                    t = 1 + R(1, 1) - R(2, 2) - R(3, 3);
                    quat = [R(3, 2) - R(2, 3), t, R(1, 2) + R(2, 1), R(3, 1) + R(1, 3)];
                else
                    t = 1 - R(1, 1) + R(2, 2) - R(3, 3);
                    quat = [R(1, 3) - R(3, 1), R(1, 2) + R(2, 1), t, R(2, 3) + R(3, 2)];
                end
            else
                if R(1, 1) < -R(2, 2)
                    t = 1 - R(1, 1) - R(2, 2) + R(3, 3);
                    quat = [R(2, 1) - R(1, 2), R(3, 1) + R(1, 3), R(2, 3) + R(3, 2), t];
                else
                    t = 1 + R(1, 1) + R(2, 2) + R(3, 3);
                    quat = [t, R(3, 2) - R(2, 3), R(1, 3) - R(3, 1), R(2, 1) - R(1, 2)];
                end
            end
            quat = quat * 0.5 / sqrt(t);

        end
        
        function as = GetAirVelocity(vel, wind_vel)
            as = wind_vel - vel;
        end
        
        function f = GetWindForce(air_vel, eff_area)
            f = 1/2 * physics.AirDensity * (air_vel.^2) * eff_area;
        end
        
        function [vec_out, rot_ic] = ApplyContactConstraints(vec_in, contact_normal, free_mat, constraint_vec, rot_to_i)
        % In the constraint vector, for each element of input vector in the contact frame we apply: 
        % 0: no constraint, 1: can only be positive, -1: can only be negative
        % rot_to_i is the vertically stacked rotation matrices to transform
        % the input vector to the inertial frame (e.g. [rot_ib; eye(3)] for
        % vec_in = [omega_b; velocity_i] input)

            % Find the rotation from the inertial to contact
            rot_ic = vrrotvec2mat(vrrotvec([1; 0; 0], contact_normal));

            % Create the block diagonal for rotations to the contact frame
            blk_c = [];
            blk_c_rev = [];
            if size(rot_to_i, 1) == 3
                blk_c = rot_ic' * rot_to_i;
                blk_c_rev = blk_c';
            elseif size(rot_to_i, 1) == 6
                % Note: 
                % For some weird reason, in R2019b this method of creating block 
                % diagonals is faster than [a, zero(3); zero(3), b] and much faster 
                % than using blkdiag function
                blk_c = eye(6);
                blk_c(1:3, 1:3) = rot_ic' * rot_to_i(1:3, :);
                blk_c(4:6, 4:6) = rot_ic' * rot_to_i(4:6, :);
                blk_c_rev = eye(6);
                blk_c_rev(1:3, 1:3) = blk_c(1:3, 1:3)';
                blk_c_rev(4:6, 4:6) = blk_c(4:6, 4:6)';
            end
            
            % The input vector transformed to the contact frame
            vec_c = blk_c * vec_in;

            % The free vector applied, which only keeps the
            % free-to-move directions and zeros the rest
            vec_free_c = free_mat * vec_c;

            % Apply the constraint matrix
            vec_free_c(vec_c .* constraint_vec < 0) = 0;

            % Transform the vector back to the original frame
            vec_out = blk_c_rev * vec_free_c;
        end
        
        function friction_force = ApplyContactFriction(force, vel, contact_normal, ...
                friction_coef, rot_to_i)

            % Find the rotation from the inertial to contact
            rot_ic = vrrotvec2mat(vrrotvec([1; 0; 0], contact_normal));

            % Create the block diagonal for rotations to the contact frame
            rot_to_c = rot_ic' * rot_to_i;
            
            % The input vectors transformed to the contact frame
            force_c = rot_to_c * force;
            vel_c = rot_to_c * vel;
            vel_c(1) = 0;

            if force_c(1) > 0
                friction_force = zeros(3, 1);
                return;
            end
            
            % Get the lateral motion direction
            vel_normalized = vel_c / norm(vel_c);

            % Calculate the friction vector
            friction_mag = abs(force_c(1)) * friction_coef;
            friction_c = -friction_mag * vel_normalized;
            
            % If the motion is too small (almost static), the friction will be at most the
            % wrench magnitude, otherwise it is in the direction against the motion
            if norm(vel_c) < 5e-4
                lat_force = [0; force_c(2:3)];
                if friction_mag > norm(lat_force)
                    friction_c = -lat_force;
                end
            end
            
            % Transform the vector back to the original frame
            friction_force = rot_to_c' * friction_c;
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

function [r1,r2,r3] = threeaxisrot(r11, r12, r21, r31, r32)
    % find angles for rotations about X, Y, and Z axes
    r1 = atan2( r11, r12 );
    r21(r21 < -1) = -1;
    r21(r21 > 1) = 1;
    r2 = asin( r21 );
    r3 = atan2( r31, r32 );
end
