clear all
close all

addpath('simulator');


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

c = controller(m);
sim = simulation(m, c);

sim.Controller.AttitudeController.SetPID(10, 0.3, 10);

pos = [0; 0; 0];
vel = [10; 20; 0];
rpy = [0; 0; 0];
omega = [0; 0; 0];
sim.Multirotor.SetInitialState(pos, vel, rpy, omega);
sim.SetTotalTime(10);
sim.SimulateAttitudeResponse([-10; 0; 30], true);

sim.Controller.PositionController.SetPID(5, 0.3, 5);
%figure; 
%sim.SimulatePositionResponse([100; 100; -100], 100, true);

graphics.PlotSignalsByName(3, {'pos', 'accel', 'rpy', 'rpy dot', 'sat'}, true);

