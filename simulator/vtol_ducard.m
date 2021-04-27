classdef vtol < multirotor
    properties
        WingDirections = {[0; 1; 0], [0; -1; 0]};
        WingLengths = [1; 1]; % in meters
        WingSurfaceArea = 0.44; % in m^2
        AngleOfIncident = 10; % in degrees
        Wingspan = 2; % in meters
        MeanChord = 0.22; % in meters
        C_A = 0.01; %Aileron constant
        C_E = 0.01; %Elevator constant
        C_R = 0.01; %Rudder constant
        r_w = 0.0625; %wing center of lift wrt, center of gravity
        r_t = 0.6385; %tail center of lift wrt, center of gravity
        S_A = ; %aileron surface area
        
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
        
        function wrench = CalcGeneratedWrench(obj, plantinput)
            wrench = CalcGeneratedWrench@multirotor(obj, plantinput);
            %wrench = zeros(6, 1);
            %wrench(6) = wrench(6) - 0.4 * physics.AirDensity * 0.52 / 2 * norm(obj.State.Velocity(1:2))^2; % Add vertical lift
            obj.CalcAerodynamicMoment(obj.State.AirVelocity)
%             wrench(1:3) = wrench(1:3) + obj.CalcAerodynamicMoment(obj.State.AirVelocity);
            wrench(4:6) = wrench(4:6) + obj.CalcAerodynamicForce(obj.State.AirVelocity);
            wrench(6) = 0.8*wrench(6)
        end
        
        function new_state = CalcNextState(obj, wrench, tf_sensor_wrench, ...
                wind_force, plantinput, dt, is_collision, collision_normal, ...
                air_velocity)
            
            new_state = CalcNextState@multirotor(obj, wrench, tf_sensor_wrench, ...
                wind_force, plantinput, dt, is_collision, collision_normal, air_velocity);
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
        function moment = CalcAerodynamicMoment(obj, Va_i)
            [rbw, alpha, ~] = obj.CalcWindToBodyRotation(Va_i);
            q_bar = (Va_i' * Va_i) * physics.AirDensity / 2;
            
            c_i = get_ci(alpha);
            c_m = get_cm(alpha);
            c_n = get_cn(alpha);
            
            roll_moment = q_bar * obj.WingSurfaceArea * obj.Wingspan * c_i;
            pitch_moment = q_bar * obj.WingSurfaceArea * obj.MeanChord * c_m;
            yaw_moment = q_bar * obj.WingSurfaceArea * obj.Wingspan * c_n;
            
            moment = rbw * [roll_moment; pitch_moment; yaw_moment];
        end
        function moment = CalcDeflectionMoment(obj, Va_i)
            [rbw, alpha, ~] = obj.CalcWindToBodyRotation(Va_i);
            q_bar = (Va_i' * Va_i) * physics.AirDensity / 2;
            
            c_i = get_ci(alpha);
            c_m = get_cm(alpha);
            c_n = get_cn(alpha);
            
            roll_moment = q_bar * obj.WingSurfaceArea * obj.Wingspan * c_i;
            pitch_moment = q_bar * obj.WingSurfaceArea * obj.MeanChord * c_m;
            yaw_moment = q_bar * obj.WingSurfaceArea * obj.Wingspan * c_n;
            
            moment = rbw * [roll_moment; pitch_moment; yaw_moment];
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
    data = load('C_l');
    cl = fixpt_interp1([-8.5:.25:13.75, 14.5,14.75, 15],data.C_l,alpha,sfix(8),2^-3,sfix(16), 2^-14,'Nearest');
end
function cd = get_cd(alpha)
    data = load('C_d');
    cd = fixpt_interp1([-8.5:.25:13.75, 14.5,14.75, 15],data.C_d,alpha,sfix(8),2^-3,sfix(16), 2^-14,'Nearest');
end
function cm = get_cm(alpha)
data = load('C_m');
cm = fixpt_interp1([-8.5:.25:13.75, 14.5,14.75, 15],data.C_m,alpha,sfix(8),2^-3,sfix(16), 2^-14,'Nearest');
end