%% Visualizer for multirotors
% This file visualizes the multirotor geometry
% Author: Azarakhsh Keipour (keipour@gmail.com)
% Last updated: June 22, 2020
function visualize_multirotor(m, plot_axes_only, plot_only)
    
    if nargin < 3
        plot_only = false;
    end

    % Visualization settings
    box_height = 0.1; % in meters
    axis_arrow_size = 0.3; % in meters
    plot_title = 'Your Cool Multirotor';
    arm_labels_on = ~plot_only;
    motor_size = 0.02; % in meters -- set to zero if don't want motors
    lighting_on = ~plot_only; % turn on the special lighting
    rotor_diameter = 12; % in inches

    % Initialization
    num_of_rotors = m.NumOfRotors;

    payload_size = m.PayloadRadius;
    arm_lengths = cell2mat(cellfun(@(s)s.ArmLength', m.Rotors, 'uni', 0));
    arm_angles = cell2mat(cellfun(@(s)s.ArmAngle', m.Rotors, 'uni', 0)) * pi / 180;
    phi_dihs = cell2mat(cellfun(@(s)s.DihedralAngle', m.Rotors, 'uni', 0)) * pi / 180;
    [~, arms_order] = sort(arm_angles);
    
    % Calculate the rotor coordinates (which also serve as the end points for arms)
    X_rotors = arm_lengths .* cos(phi_dihs) .* cos(arm_angles);
    Y_rotors = arm_lengths .* cos(phi_dihs) .* sin(arm_angles);
    Z_rotors = arm_lengths .* sin(-phi_dihs);
    
    % Visualize the geometry

    % Draw the arms
    for i = 1 : num_of_rotors
        hold on
        plotArm([X_rotors(i); Y_rotors(i); Z_rotors(i)], m.Rotors{i}.R_BR * [0;0;-1], i, arm_labels_on, plot_axes_only, motor_size);
    end

    % Draw the rotors
    for i = 1 : num_of_rotors
        hold on
        if plot_axes_only == false
            plotRotor([X_rotors(i); Y_rotors(i); Z_rotors(i)], m.Rotors{i}.R_BR * [0;0;-1], ...
                m.Rotors{i}.RotationDirection, axis_arrow_size, motor_size, rotor_diameter, plot_only);
        else
            plotAxes([X_rotors(i); Y_rotors(i); Z_rotors(i)], m.Rotors{i}.R_BR,  axis_arrow_size / 2);
        end
    end

    % Draw the central payload box
    hold on
    if plot_axes_only == false
        plotBox(X_rotors, Y_rotors, Z_rotors, arm_lengths, arms_order, payload_size, box_height);
    else
        [sx, sy, sz] = sphere;
        sphere_size = axis_arrow_size / 10;
        sx = sx * sphere_size;
        sy = sy * sphere_size;
        sz = sz * sphere_size;
        surf(sx, sy, sz);
        plotAxes(zeros(3, 1), eye(3),  axis_arrow_size / 2);
    end

    if m.HasEndEffector()
        plotEndEffectorArm(m.EndEffector.BasePosition, ...
            m.EndEffector.EndEffectorPosition, ...
            -m.EndEffector.Direction, ...
            plot_axes_only, 0.05);
    end
    
    % Make the plot more presentable

    if plot_only == false
        % Rotate the axes for better visualization
        set(gca, 'Xdir', 'reverse')
        set(gca, 'Zdir', 'reverse')

        % Equalize the axes scales
        axis equal;

        % Add title and axis labels
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        title(plot_title);
    end

    % Change lighting
    if lighting_on
        camlight
        lighting gouraud %phong
    end
    
    hold off
end

%% Helper functions

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
    
    % Draw the forward triangle
    tri_xs = [box_size / 2; -box_size / 3; -box_size / 3];
    tri_ys = [0; -box_size / 6; box_size / 6];
    tri_zs = ones(3, 1) * (min(box_zs) - box_height / 2 - 0.001);
    fill3(tri_xs, tri_ys, tri_zs, 'red');
end

function plotEndEffectorArm(start_pos, end_pos, z_axis, plot_axes_only, ee_size)
    ee_color = [0.4940, 0.1840, 0.5560];
    line_width = 6;
    if plot_axes_only
        line_width = 1;
    end
    plot3([start_pos(1), end_pos(1)], [start_pos(2), end_pos(2)], [start_pos(3), end_pos(3)], 'Color', ee_color, 'LineWidth', line_width);
    
    circlePlane3D(end_pos, z_axis, ee_size, 0.005, false, ee_color, 0, 0, true);    
end

function plotArm(position, z_axis, num, arm_labels_on, plot_axes_only, motor_size)
    line_width = 3;
    if plot_axes_only
        line_width = 1;
    end
    plot3([0, position(1)], [0, position(2)], [0, position(3)], 'k', 'LineWidth', line_width);
    if arm_labels_on
        label_dist = 0.02;
        dp = -(motor_size + label_dist) * z_axis;
        if plot_axes_only
            dp = -dp;
        end
        text(position(1) + dp(1), position(2) + dp(2), position(3) + dp(3), num2str(num), 'Interpreter', 'none');
    end
end

function plotRotor(position, axis, direction, arrow_size, motor_size, rotor_diameter, no_axes)
    rotor_size = rotor_diameter * 0.0254 / 2; 
    rotor_color = [0.4, 0.4, 1]; % CW
    if direction == 1
        rotor_color = [0.4, 1, 0.4];
    end
    motor_color = 'black';
    dmot = motor_size * axis;
    pos_m1 = position - dmot;
    pos_m2 = position + dmot;
    plot3([pos_m1(1), pos_m2(1)], [pos_m1(2), pos_m2(2)], [pos_m1(3), pos_m2(3)], 'Color', motor_color, 'LineWidth', 10);
    circlePlane3D(pos_m2, axis, rotor_size, 0.005, ~no_axes, rotor_color, arrow_size, direction, no_axes);
end

function plotAxes(position, Rotation, arrow_size)
    colors = {'green', 'blue', 'red'};
    labels = {'$\hat{x}$', '$\hat{y}$', '$\hat{z}$'};
    label_dist = 0.02;
    for i = 1 : 3
        end_pos = position + arrow_size*Rotation(:, i);
        label_pos = end_pos + label_dist * Rotation(:, i);
        arrow3d([position(1) end_pos(1)], [position(2) end_pos(2)], [position(3) end_pos(3)], 0.8, 0.005, 0.01, colors{i});
        text(label_pos(1), label_pos(2), label_pos(3), labels{i}, 'Interpreter', 'latex');
    end
end

%% Draw a 3-D circle
% Downloaded from https://www.mathworks.com/matlabcentral/fileexchange/37879-circle-plane-in-3d
% With some modifications and bug fixes
function H = circlePlane3D( center, normal, radious, theintv, normalon, color, arrow_size, direction, no_axes)
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
    if no_axes == false
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
end

%% Draw a 3-D arrow
function [h]=arrow3d(x,y,z,head_frac,radii,radii2,colr)
%
% The function plotting 3-dimensional arrow
%
% h=arrow3d(x,y,z,head_frac,radii,radii2,colr)
%
% The inputs are:
%       x,y,z =  vectors of the starting point and the ending point of the
%           arrow, e.g.:  x=[x_start, x_end]; y=[y_start, y_end];z=[z_start,z_end];
%       head_frac = fraction of the arrow length where the head should  start
%       radii = radius of the arrow
%       radii2 = radius of the arrow head (defult = radii*2)
%       colr =   color of the arrow, can be string of the color name, or RGB vector  (default='blue')
%
% The output is the handle of the surfaceplot graphics object.
% The settings of the plot can changed using: set(h, 'PropertyName', PropertyValue)
%
% example #1:
%        arrow3d([0 0],[0 0],[0 6],.5,3,4,[1 0 .5]);
% example #2:
%        arrow3d([2 0],[5 0],[0 -6],.2,3,5,'r');
% example #3:
%        h = arrow3d([1 0],[0 1],[-2 3],.8,3);
%        set(h,'facecolor',[1 0 0])
% 
% Written by Moshe Lindner , Bar-Ilan University, Israel.
% July 2010 (C)

if nargin==5
    radii2=radii*2;
    colr='blue';
elseif nargin==6
    colr='blue';
end
if size(x,1)==2
    x=x';
    y=y';
    z=z';
end

x(3)=x(2);
x(2)=x(1)+head_frac*(x(3)-x(1));
y(3)=y(2);
y(2)=y(1)+head_frac*(y(3)-y(1));
z(3)=z(2);
z(2)=z(1)+head_frac*(z(3)-z(1));
r=[x(1:2)',y(1:2)',z(1:2)'];

N=50;
dr=diff(r);
dr(end+1,:)=dr(end,:);
origin_shift=(ones(size(r))*(1+max(abs(r(:))))+[dr(:,1) 2*dr(:,2) -dr(:,3)]);
r=r+origin_shift;

normdr=(sqrt((dr(:,1).^2)+(dr(:,2).^2)+(dr(:,3).^2)));
normdr=[normdr,normdr,normdr];
dr=dr./normdr;
Pc=r;
n1=cross(dr,Pc);
normn1=(sqrt((n1(:,1).^2)+(n1(:,2).^2)+(n1(:,3).^2)));
normn1=[normn1,normn1,normn1];
n1=n1./normn1;
P1=n1+Pc;

X1=[];Y1=[];Z1=[];
j=1;
for theta=([0:N])*2*pi./(N);
    R1=Pc+radii*cos(theta).*(P1-Pc) + radii*sin(theta).*cross(dr,(P1-Pc)) -origin_shift;
    X1(2:3,j)=R1(:,1);
    Y1(2:3,j)=R1(:,2);
    Z1(2:3,j)=R1(:,3);
    j=j+1;
end

r=[x(2:3)',y(2:3)',z(2:3)'];

dr=diff(r);
dr(end+1,:)=dr(end,:);
origin_shift=(ones(size(r))*(1+max(abs(r(:))))+[dr(:,1) 2*dr(:,2) -dr(:,3)]);
r=r+origin_shift;

normdr=(sqrt((dr(:,1).^2)+(dr(:,2).^2)+(dr(:,3).^2)));
normdr=[normdr,normdr,normdr];
dr=dr./normdr;
Pc=r;
n1=cross(dr,Pc);
normn1=(sqrt((n1(:,1).^2)+(n1(:,2).^2)+(n1(:,3).^2)));
normn1=[normn1,normn1,normn1];
n1=n1./normn1;
P1=n1+Pc;

j=1;
for theta=([0:N])*2*pi./(N);
    R1=Pc+radii2*cos(theta).*(P1-Pc) + radii2*sin(theta).*cross(dr,(P1-Pc)) -origin_shift;
    X1(4:5,j)=R1(:,1);
    Y1(4:5,j)=R1(:,2);
    Z1(4:5,j)=R1(:,3);
    j=j+1;
end

X1(1,:)=X1(1,:)*0 + x(1);
Y1(1,:)=Y1(1,:)*0 + y(1);
Z1(1,:)=Z1(1,:)*0 + z(1);
X1(5,:)=X1(5,:)*0 + x(3);
Y1(5,:)=Y1(5,:)*0 + y(3);
Z1(5,:)=Z1(5,:)*0 + z(3);

h=surf(X1,Y1,Z1,'facecolor',colr,'edgecolor','none');
end
