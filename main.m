%% Prepare the environment

clear all
close all

addpath('simulator');

%% Define the hardware architecture
%m = robots.floating_hex();
%m = robots.quadrotor();
m = robots.tilted_hex(true);

%m.Visualize();
%m.VisualizeAxes();
%m.AnalyzeDynamicManipulability(2, 2);

%% Prepare the simulation

c = controller(m);
sim = simulation(m, c);

% Initial multirotor state
pos = [0; 0; 0];
vel = [0; 0; 0];
rpy = [0; 0; 0];
omega = [0; 0; 0];
sim.Multirotor.SetInitialState(pos, vel, rpy, omega);

sim.SetTotalTime(20);

%% Prepare the controller

sim.Controller.AttitudeController.SetPID(60, 0, 20);
sim.Controller.PositionController.SetPID(3, 0, 7);
sim.Controller.PositionController.AttitudeType = attitude_types.ZeroTilt;

%% Get the controller response(s)

% Attitude response
%figure; 
%sim.SimulateAttitudeResponse([0; 0; -90], true);

% Position response
figure;
sim.SimulatePositionResponse([20; 0; 0], 90, true);

% Additional plots
graphics.PlotSignalsByName(3, {'pos', 'vel', 'accel', 'rpy', 'euler deriv', 'ang accel'}, true);
