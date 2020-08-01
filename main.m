%% Prepare the environment

clear all
close all

addpath('simulator');

%% Define the hardware architecture

% RotorPlacementAngles = [45, 135, 225, 315];
% RotorRotationDirections = [-1, 1, -1, 1];
% RotorDihedralAngle = 0;
% RotorSidewardAngle = 0;
% RotorInwardAngle = 0;

RotorPlacementAngles = [30, 90, 150, 210, 270, 330];
RotorRotationDirections = [-1, 1, -1, 1, -1, 1];
RotorDihedralAngle = 0;
RotorSidewardAngle = [-30, 30, -30, 30, -30, 30];
RotorInwardAngle = 0;

m = multirotor(RotorPlacementAngles, RotorRotationDirections);
m.SetRotorAngles(RotorInwardAngle, RotorSidewardAngle, RotorDihedralAngle);
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

%% Get the controller response(s)

% Attitude response
sim.SetTotalTime(20);
sim.Controller.AttitudeController.SetPID(60, 0, 20);
%figure; 
%sim.SimulateAttitudeResponse([-10; 0; 30], true);

% Position response
sim.SetTotalTime(20);
sim.Controller.PositionController.SetPID(3, 0, 7);
figure;
sim.SimulatePositionResponse([100; 100; -100], 100, true);

% Additional plots
graphics.PlotSignalsByName(3, {'pos', 'vel', 'accel', 'rpy', 'ang accel'}, true);
