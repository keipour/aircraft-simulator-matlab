%% Inertia estimator for multirotors
% Calculate the inertia tensor of the input multirotor
% Author: Azarakhsh Keipour (keipour@gmail.com)
% Last updated: June 22, 2020
function inertia_tensor = calc_inertia(m)

    % Initialization
    num_rotors = m.NumOfRotors;
    arm_lengths = cell2mat(cellfun(@(s)s.ArmLength', m.Rotors, 'uni', 0));
    arm_angles = cell2mat(cellfun(@(s)s.ArmAngle', m.Rotors, 'uni', 0)) * pi / 180;
    phi_dihs = cell2mat(cellfun(@(s)s.DihedralAngle', m.Rotors, 'uni', 0)) * pi / 180;
    mass_arms = cell2mat(cellfun(@(s)s.ArmMass', m.Rotors, 'uni', 0));
    mass_motors = cell2mat(cellfun(@(s)s.MotorMass', m.Rotors, 'uni', 0));
    mass_payload = m.Mass - sum(mass_arms) - sum(mass_motors);
    payload_radius = m.PayloadRadius;

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
        arm_tensor{i} = rod_inertia(mass_arms(i), 0, 0, 0, X_rotors(i), Y_rotors(i), Z_rotors(i));
    end

    % Calculate the overall tensor as the sum of all the tensors
    inertia_tensor = payload_tensor;
    for i = 1 : num_rotors
        inertia_tensor = inertia_tensor + rotor_tensor{i} + arm_tensor{i};
    end

end

%% Calculate the intertia tensor of a solid sphere around the center
function inertia_tensor = solid_sphere_inertia(radius, mass)
    inertia_tensor = (2/5 * mass * radius^2) * eye(3);
end

%% Calculate the intertia tensor of a rod from point 1 to 2 around the center
function inertia_tensor = rod_inertia(mass, x1, y1, z1, x2, y2, z2)
    N = 1e5; % Numerical calculation fidelity

    x_lin = linspace(x1, x2, N);
    y_lin = linspace(y1, y2, N);
    z_lin = linspace(z1, z2, N);
    dm = mass / length(x_lin);

    % Calculate the tensor by summing the point masses
    inertia_tensor  = zeros(3);
    for i = 1 : length(x_lin)
        inertia_tensor = inertia_tensor + point_mass_inertia(dm, x_lin(i), y_lin(i), z_lin(i));
    end
end

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
