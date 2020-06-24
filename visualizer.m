%% Visualizer for multirotors
% This file visualizes the multirotor geometry
% Author: Azarakhsh Keipour (keipour@gmail.com)
% Last updated: June 22, 2020

%% Initialize
function visualizer(m)

    % Create the new figure
    figure;

    % Visualization settings
    box_height = 0.1; % in meters
    axis_arrow_size = 0.3; % in meters
    plot_title = 'Your Cool Multirotor';
    arm_labels_on = true;
    motor_size = 0.05; % in meters -- set to zero if don't want motors
    lighting_on = true; % turn on the special lighting
    rotor_diameter = 12; % in inches

    % Initialization
    num_of_rotors = m.GetNumOfRotors();

    payload_size = m.PayloadRadius;
    arm_lengths = cell2mat(arrayfun(@(s)s.ArmLength', m.Rotors, 'uni', 0));
    arm_angles = cell2mat(arrayfun(@(s)s.ArmAngle', m.Rotors, 'uni', 0)) * pi / 180;
    phi_dihs = cell2mat(arrayfun(@(s)s.DihedralAngle', m.Rotors, 'uni', 0)) * pi / 180;
    [~, arms_order] = sort(arm_angles);
    
    % Calculate the rotor coordinates (which also serve as the end points for arms)
    X_rotors = arm_lengths .* cos(phi_dihs) .* cos(arm_angles);
    Y_rotors = arm_lengths .* cos(phi_dihs) .* sin(arm_angles);
    Z_rotors = arm_lengths .* sin(-phi_dihs);
    
    %% Visualize the geometry

    % Draw the arms
    for i = 1 : num_of_rotors
        hold on
        plotArm([X_rotors(i); Y_rotors(i); Z_rotors(i)], i, arm_labels_on, motor_size);
    end

    % Draw the rotors
    for i = 1 : num_of_rotors
        hold on
        plotRotor([X_rotors(i); Y_rotors(i); Z_rotors(i)], m.Rotors(i).R * [0;0;-1], ...
            m.Rotors(i).RotationDirection, axis_arrow_size, motor_size, rotor_diameter);
    end

    % Draw the central payload box
    hold on
    plotBox(X_rotors, Y_rotors, Z_rotors, arm_lengths, arms_order, payload_size, box_height);

    %% Make the plot more presentable

    % Rotate the axes for better visualization
    set(gca, 'Xdir', 'reverse')
    set(gca, 'Zdir', 'reverse')

    % Equalize the axes scales
    axis equal;

    % Make the 3D plot a sqaure box
    % xl = xlim;
    % yl = ylim;
    % zl = zlim;
    % min_limit = min([xl(1), yl(1), zl(1)]);
    % max_limit = max([xl(2), yl(2), zl(2)]);
    % xlim([min_limit max_limit]);
    % ylim([min_limit max_limit]);
    % zlim([min_limit max_limit]);

    % Add title and axis labels
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title(plot_title);


    % Change lighting
    if lighting_on
        camlight
        lighting gouraud %phong
    end
end
%% Functions

function plotBox(X_rotors, Y_rotors, Z_rotors, arm_lengths, arms_order, box_size, box_height)
    num_of_rotors = length(arms_order);
    box_xs = (X_rotors * box_size) ./ arm_lengths;
    box_ys = (Y_rotors * box_size) ./ arm_lengths;
    box_zs = (Z_rotors * box_size) ./ arm_lengths;
    fill3(box_xs(arms_order), box_ys(arms_order), box_zs(arms_order) + box_height / 2, 'cyan');
    fill3(box_xs(arms_order), box_ys(arms_order), box_zs(arms_order) - box_height / 2, 'cyan');
    for i = 1 : num_of_rotors
        j = mod(i, num_of_rotors) + 1;
        xs = [box_xs(arms_order(i)) box_xs(arms_order(i)) box_xs(arms_order(j)) box_xs(arms_order(j))];
        ys = [box_ys(arms_order(i)) box_ys(arms_order(i)) box_ys(arms_order(j)) box_ys(arms_order(j))];
        zs = [(box_zs(arms_order(i)) + box_height / 2) (box_zs(arms_order(i)) - box_height / 2) (box_zs(arms_order(j)) - box_height / 2) (box_zs(arms_order(j)) + box_height / 2)];
        fill3(xs, ys, zs, 'cyan');
    end
end

function plotArm(position, num, arm_labels_on, motor_size)
    plot3([0, position(1)], [0, position(2)], [0, position(3)], 'k', 'LineWidth', 3);
    if arm_labels_on
        text(position(1), position(2), position(3) + .05 + motor_size, num2str(num), 'Interpreter', 'none');
    end
end

function plotRotor(position, axis, direction, arrow_size, motor_size, rotor_diameter)
    rotor_size = rotor_diameter * 0.0254 / 2; 
    rotor_color = [0.4, 0.4, 1]; % CW
    if direction == 1
        rotor_color = [0.4, 1, 0.4];
    end
    motor_color = 'black';
    plot3([position(1), position(1)], [position(2), position(2)], [position(3) - motor_size, position(3) + motor_size], 'Color', motor_color, 'LineWidth', 10);
    position(3) = position(3) - motor_size;
    circlePlane3D(position, axis, rotor_size, 0.005, 1, rotor_color, arrow_size, direction);
end

% Downloaded from https://www.mathworks.com/matlabcentral/fileexchange/37879-circle-plane-in-3d
% With some modifications and bug fixes
function H = circlePlane3D( center, normal, radious, theintv, normalon, color, arrow_size, direction)
    %CIRCLEPLANE3D Summary of this function goes here
    %--------------------------------------------------------------------------
    %Generate a circle plane in 3D with the given center and radious
    %The plane is defined by the normal vector
    %theintv is the interval theta which allow you to control your polygon
    %shape
    % Example:,
    %
    %   circlePlane3D([0 0 0], [1 -1 2], 5, 0.2, 1, [0 0 1], '-'); 
    %   circlePlane3D([3 3 -3],[0 1 1], 3, 0.1, 1, 'y', '-');
    %   
    %   Cheng-Yuan Wu <ieda_wind@hotmail.com>
    %   Version 1.00
    %   Aug, 2012
    %--------------------------------------------------------------------------

    % The disc plotting has a bug when the normal is completely in Z direction
    if normal(1) == 0 && normal(2) == 0 
        normal(1) = normal(1) + 1e-8;
    end
    %generate circle polygon
    t = 0:theintv:2*pi;
    x = radious*cos(t);
    y = radious*sin(t);
    z = zeros(size(x));
    %compute rotate theta and axis
    zaxis = [0 0 1];
    normal = normal/norm(normal);
    ang = acos(dot(zaxis,normal));
    axis = cross(zaxis, normal)/norm(cross(zaxis, normal));
    % A skew symmetric representation of the normalized axis 
    axis_skewed = [ 0 -axis(3) axis(2) ; axis(3) 0 -axis(1) ; -axis(2) axis(1) 0]; 
    % Rodrigues formula for the rotation matrix 
    R = eye(3) + sin(ang)*axis_skewed + (1-cos(ang))*axis_skewed*axis_skewed;
    fx = R(1,1)*x + R(1,2)*y + R(1,3)*z;
    fy = R(2,1)*x + R(2,2)*y + R(2,3)*z;
    fz = R(3,1)*x + R(3,2)*y + R(3,3)*z;
    %translate center
    fx = fx+center(1);
    fy = fy+center(2);
    fz = fz+center(3);
    H = fill3(fx, fy, fz, color);
    if normalon == 1
        hold on;
        normal_scaled = normal * arrow_size;
        %plot3([center(1) center(1)+normal(1)],[center(2) center(2)+normal(2)],[center(3) center(3)+normal(3)],'-');
        H = arrow3d([center(1) center(1)+normal_scaled(1)],[center(2) center(2)+normal_scaled(2)],[center(3) center(3)+normal_scaled(3)], 0.8, 0.01, 0.02, 'red');
    end
    
    % draw the rotations with arrows
    raise_amount = 0.1;
    arc_length = 0.75; % * 2pi
    n_points = floor(length(t) * arc_length);
    X = (fx(1 : n_points) + center(1)) / 2 + raise_amount * normal(1);
    Y = (fy(1 : n_points) + center(2)) / 2 + raise_amount * normal(2);
    Z = (fz(1 : n_points) + center(3)) / 2 + raise_amount * normal(3);
    line(X,Y,Z, 'LineWidth', 2, 'Color', 'magenta');
    if direction == 1
        arrow3d([X(end-50) X(end)], [Y(end-50) Y(end)], [Z(end-50) Z(end)], 0, 0.005, 0.01, 'red');
        %quiver3(X(end-1), Y(end-1), Z(end-1), X(end)-X(end-1), Y(end)-Y(end-1), Z(end)-Z(end-1),1, 'LineWidth', 10, 'Color', 'magenta');
    else
        arrow3d([X(50) X(1)], [Y(50) Y(1)], [Z(50) Z(1)], 0, 0.005, 0.01, 'red');
        %quiver3(X(2), Y(2), Z(2), X(1)-X(2), Y(1)-Y(2), Z(1)-Z(2), 1, 'LineWidth', 10, 'Color', 'magenta');
    end
end
