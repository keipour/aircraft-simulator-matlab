classdef multirotor < handle
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
        InitialState                % Initial state
        State                       % The current state
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
                obj.Rotors{i} = rotor.Create();
                obj.Rotors{i} = rotor.SetArmAngle(obj.Rotors{i}, ArmAngles(i));
                obj.Rotors{i}.RotationDirection = RotationDirections(i);
            end
            
            obj.InitialState = state.Create();
            obj.State = state.Create();
            
            obj.UpdateStructure();
        end
        
        function set.Rotors(obj, value)
            obj.Rotors = value;
            obj.UpdateNumOfRotors();
        end
        
        function SetInitialState(obj, pos, vel, rpy, omega)
            obj.InitialState.Position = pos;
            obj.InitialState.Velocity = vel;
            obj.InitialState.RPY = rpy;
            obj.InitialState.Omega = omega;
            
            obj.State.Position = pos;
            obj.State.Velocity = vel;
            obj.State.RPY = rpy;
            obj.State.Omega = omega;
        end
        
        function SetRotorAngles(obj, RotorInwardAngles, RotorSidewardAngles, RotorDihedralAngles)
            % Set the angles of the rotors
            % Inputs can be scalar or an array of he same length as 
            % the number of rotors.

            if length(RotorInwardAngles) == 1
                RotorInwardAngles = ones(obj.NumOfRotors, 1) * RotorInwardAngles;
            end
            if length(RotorSidewardAngles) == 1
                RotorSidewardAngles = ones(obj.NumOfRotors, 1) * RotorSidewardAngles;
            end
            if length(RotorDihedralAngles) == 1
                RotorDihedralAngles = ones(obj.NumOfRotors, 1) * RotorDihedralAngles;
            end
            
            % Assign the values
            for i = 1 : obj.NumOfRotors
                obj.Rotors{i} = rotor.SetInwardAngle(obj.Rotors{i}, RotorInwardAngles(i));
                obj.Rotors{i} = rotor.SetSidewardAngle(obj.Rotors{i}, RotorSidewardAngles(i));
                obj.Rotors{i} = rotor.SetDihedralAngle(obj.Rotors{i}, RotorDihedralAngles(i));
            end
            
            % Update the structure
            obj.UpdateStructure();
        end
        
        function UpdateState(obj, RotorSpeedsSquared, dt)
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
        
        function set.I(obj, value)
            obj.I = value;
            obj.UpdateI_inv();
        end
        
        function UpdateStructure(obj)
            obj.I = obj.EstimateInertia();
            obj.UpdateNumOfRotors();
        end
        
        % Estimate the inertia tensor of the multirotor
        function inertia_tensor = EstimateInertia(m)

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
        
        function Visualize(obj)
            visualize_multirotor(obj);
        end
       
    end
    
    %% Private Methods
    methods(Access=protected)
        
        function UpdateNumOfRotors(obj)
            obj.NumOfRotors = length(obj.Rotors);
        end
        
        function UpdateI_inv(obj)
            obj.I_inv = pinv(obj.I);
        end

        function F = GetGravityForce(obj)
            F = obj.Gravity * obj.Mass;
        end
        
        function F = GetThrustForce(obj, RotorSpeedsSquared)
            F = zeros(3, 1);
            for i = 1 : obj.NumOfRotors
               F = F + rotor.GetThrustForce(obj.Rotors{i}, RotorSpeedsSquared(i));
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
                F = rotor.GetThrustForce(obj.Rotors{i}, RotorSpeedsSquared(i));
                M = M + cross(r, F);
            end
        end
        
        function M = GetReactionMoment(obj, RotorSpeedsSquared)
            M = zeros(3, 1);
            for i = 1 : obj.NumOfRotors
               M = M + rotor.GetReactionMoment(obj.Rotors{i}, RotorSpeedsSquared(i));
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

%% Visualizer for multirotors
% This file visualizes the multirotor geometry
% Author: Azarakhsh Keipour (keipour@gmail.com)
% Last updated: June 22, 2020
function visualize_multirotor(m)

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
    
    %% Visualize the geometry

    % Draw the arms
    for i = 1 : num_of_rotors
        hold on
        plotArm([X_rotors(i); Y_rotors(i); Z_rotors(i)], i, arm_labels_on, motor_size);
    end

    % Draw the rotors
    for i = 1 : num_of_rotors
        hold on
        plotRotor([X_rotors(i); Y_rotors(i); Z_rotors(i)], m.Rotors{i}.R * [0;0;-1], ...
            m.Rotors{i}.RotationDirection, axis_arrow_size, motor_size, rotor_diameter);
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

%% Draw a 3-D circle
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