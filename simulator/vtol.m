classdef vtol < multirotor
    properties
        WingDirections = {[0; 1; 0], [0; -1; 0]};
        WingLengths = [1; 1]; % in meters
        WingSurfaceArea = 0.44; % in m^2
        AngleOfIncident = 10; % in degrees
    end

    properties (SetAccess=protected, GetAccess=public)
        % Add your properties that can only be set internally
    end
    
    properties (SetAccess=protected, GetAccess=protected)
        % Add your properties that can only be seen internally
    end
    
    %% Public methods
    methods
        function obj = vtol(ArmAngles, RotationDirections)
            obj = obj@multirotor(ArmAngles, RotationDirections);
        end
        
        function wrench = CalcGeneratedWrench(obj, rotor_speeds_squared)
            wrench = CalcGeneratedWrench@multirotor(obj, rotor_speeds_squared);
            %wrench = zeros(6, 1);
            %wrench(6) = wrench(6) - 0.4 * physics.AirDensity * 0.52 / 2 * norm(obj.State.Velocity(1:2))^2; % Add vertical lift
            wrench(4:6) = wrench(4:6) + obj.CalcAerodynamicForce(obj.State.AirVelocity);
        end
        
        function new_state = CalcNextState(obj, wrench, tf_sensor_wrench, ...
                wind_force, RotorSpeedsSquared, dt, is_collision, collision_normal, air_velocity)
            
            new_state = CalcNextState@multirotor(obj, wrench, tf_sensor_wrench, ...
                wind_force, RotorSpeedsSquared, dt, is_collision, collision_normal, air_velocity);
            [~, alpha, beta] = CalcWindToBodyRotation(obj, air_velocity);
            new_state.AngleOfAttack = alpha;
            new_state.SideSlipAngle = beta;
        end
    end
    
    %% Private methods
    methods (Access = private)
        function force = CalcAerodynamicForce(obj, Va_i)
            [rbw, alpha, ~] = obj.CalcWindToBodyRotation(Va_i);
            q_bar = (Va_i' * Va_i) * physics.AirDensity / 2;
            
            c_y = 0; % TODO: Later
            c_x = get_cd(alpha); % TODO: Add beta-dependent part later
            c_z = get_cl(alpha);
            
            drag = q_bar * obj.WingSurfaceArea * c_x;
            lateral = q_bar * obj.WingSurfaceArea * c_y;
            lift = q_bar * obj.WingSurfaceArea * c_z;
            
            force = rbw * [drag; lateral; -lift];
        end

        function [R_BW, alpha, beta] = CalcWindToBodyRotation(obj, Va_i)
            rbi = obj.GetRotationMatrix();
            Va_b = rbi * Va_i;
            a = CalcAngleOfAttack(Va_b) + obj.AngleOfIncident;
            b = CalcSideSlipAngle(Va_b);
            R_BW = [cosd(b) * cosd(a),  -sind(b) * cosd(a),     -sind(a)
                    sind(b),            cosd(b),                0
                    cosd(b) * sind(a),  - sind(b) * sind(a),    cosd(a) ];
            if nargout > 1
                alpha = a;
                beta = b;
            end
        end
    end
    
end

%% Other function

function alpha = CalcAngleOfAttack(Va_b)
    alpha = atan2d(Va_b(3), Va_b(1));
end

function beta = CalcSideSlipAngle(Va_b)
    beta = asind(Va_b(2) / norm(Va_b));
    if isnan(beta)
        beta = 0;
    end
end

function cl = get_cl(alpha)
    alpha_points = [-20, -15, 15, 20];

    cl_points = [1.0, -1, 2.2, 1.2];

    cl = zeros(size(alpha));

    f1 = polyfit(alpha_points(1:2), cl_points(1:2), 1);
    f2 = polyfit(alpha_points(2:3), cl_points(2:3), 1);
    f3 = polyfit(alpha_points(3:end), cl_points(3:end), 1);

    for i = 1:size(alpha, 2)
        if (alpha(i) <= -15)
            cl(i) = f1(1) * alpha(i) + f1(2);
        elseif (alpha(i) >= 15)
            cl(i) = f3(1) * alpha(i) + f3(2);
        else
            cl(i) = f2(1) * alpha(i) + f2(2);
        end 
    end
end

function cd = get_cd(alpha)
    alpha_points = [-15, -5, 0, 10, 20];

    cd_points = [0.15, 0.015, 0.009, 0.019, 0.25];

    cd = zeros(size(alpha));

    f1 = polyfit(alpha_points(1:2), cd_points(1:2), 1);
    f2 = polyfit(alpha_points(2:3), cd_points(2:3), 1);
    f3 = polyfit(alpha_points(3:4), cd_points(3:4), 1);
    f4 = polyfit(alpha_points(4:end), cd_points(4:end), 1);

    for i = 1:size(alpha, 2)
        if (alpha(i) < -5)
            cd(i) = f1(1) * alpha(i) + f1(2);
        elseif (alpha(i) >= 10)
            cd(i) = f4(1) * alpha(i) + f4(2);
        elseif ((alpha(i) >= -5) && (alpha(i) < 0))
            cd(i) = f2(1) * alpha(i) + f2(2);
        else
            cd(i) = f3(1) * alpha(i) + f3(2);
        end 
    end
end