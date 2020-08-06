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
