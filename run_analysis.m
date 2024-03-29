%% Prepare the environment

clear all
close all

addpath('simulator');

%% Define the hardware architecture

% Feel free to try these:

%m = robots.floating_hex();
%m = robots.tilted_quad();
%m = robots.octorotor_assymmetric();
%m = robots.quadrotor(true);
%m = robots.vtol_custom();
m = robots.tilted_hex(true);

%% Define the current state
% table = zeros(41, 1);
% for i = 0 : 0

position = [0; 0; 0]; % Obviously, position has no effect on manipulability
velocity = [0; 0; 0]; % in m/s
rpy = [0; 0; 0]; % Roll, Pitch and Yaw in degrees
omega = [0; 0; 0]; % in rad/s

m.SetInitialState(position, velocity, rpy, omega);

%% Define the external forces

% Uncomment and change these only if you are curious, otherwise leave them
% commented

% The gravity is defined in 'physics.Gravity'
%physics.Gravity = [0; 0; 9.80665];  % in m/s^2

% The multirotor mass is defined in 'm.TotalMass'
%m.TotalMass = 7.427; % in kg

% The air density is defined in 'physics.AirDensity' (has a small effect on 
% drag, which depends on the airspeed as well)
%physics.AirDensity = 1.229;          % in kg/m^3

% Wind speed in NED frame
wind = [0; 0; 0]; % in m/s

% Calculate the wind_force applied to the multirotor
air_velocity = physics.GetAirVelocity(velocity, wind);
eff_wind_area = m.WindModel.CalcualteEffectiveArea(air_velocity, rpy);
wind_force = physics.GetWindForce(air_velocity, eff_wind_area);

% Note that there will be a non-zero wind force even if you have zero wind
% with non-zero velocity. You can just set the force to your desired value
% if you want so:
wind_force = [0; 0; 0]; % in N

%% Visualize the robot

% m.Visualize();
% m.VisualizeAxes();

%% Analyze the dynamic manipulability
% m.AnalyzeDynamicManipulability(wind_force);

% Get the range of possible angular accelerations for the given fixed
% linear accelerations (can be converted from applied forces knowing the 
% mass and gravity)
% Note that currently it can calculate the acceleration set for any given
% combination of accelerations, but the visualization only happens
% correctly if the three linear accelerations are given and the angular
% accelerations are requested. The requested dimensions are specified by
% nan values in the second input argument of the analysis function below.
fixed_lin_accel = [1.0, 0.5, 0.25];
res = m.AnalyzeDynamicManipulability6D(wind_force, [nan, nan, nan, fixed_lin_accel]);

% table(i + 21) = res.MaximumAngularAccelerationX;
% end
% disp(table)