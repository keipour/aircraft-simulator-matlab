function inertia_tensor = hex_inertia(arm_length, arm_angles, phi_dih, m_sphere, m_arm, m_rotor, r_sphere)

% Convert the angles to radians
arm_angles = arm_angles * pi / 180;
phi_dih = phi_dih * pi / 180;

% Calculate the rotor coordinates (which also serve as the end points for arms)
X_rotors = arm_length * cos(phi_dih) * cos(arm_angles);
Y_rotors = arm_length * cos(phi_dih) * sin(arm_angles);
Z_rotors = arm_length * sin(phi_dih) * ones(length(arm_angles), 6);

% Calculate the sphere tensor (the main mass around the center)
sphere_tensor = solid_sphere_inertia(r_sphere, m_sphere);

% Calculate the rotors tensor (sum of all 6 rotors as point masses)
rotor_tensor = cell(6, 1);
for i = 1 : 6
    rotor_tensor{i} = point_mass_inertia(m_rotor, X_rotors(i), Y_rotors(i), Z_rotors(i));
end

% Calculate the arm tensor (sum of all 6 arms as rods connecting center to rotors)
arm_tensor = cell(6, 1);
for i = 1 : 6
    arm_tensor{i} = rod_inertia(m_arm, 0, 0, 0, X_rotors(i), Y_rotors(i), Z_rotors(i));
end

% Calculate the overall tensor as the sum of all the tensors
inertia_tensor = sphere_tensor;
for i = 1 : 6
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
