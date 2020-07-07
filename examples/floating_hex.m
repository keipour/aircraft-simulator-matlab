clear all
close all

addpath('../simulator');


% RotorPlacementAngles = [45, 135, 225, 315];
% RotorRotationDirections = [-1, 1, -1, 1];
% 

RotorPlacementAngles = [30, 90, 150, 210, 270, 330];
RotorRotationDirections = [-1, 1, -1, 1, -1, 1];
RotorDihedralAngle = 20;
RotorSidewardAngle = [-30, 30, -30, 30, -30, 30];
RotorInwardAngle = 0;

RotorPlacementAngles = [0, 0, 90, 180, 180, 270];
RotorRotationDirections = [1, 1, 1, 1, 1, 1];
RotorDihedralAngle = [90, 0, 0, -90, 0, 0];
RotorSidewardAngle = [90, 0, 90, 90, 180, 90];
RotorInwardAngle = [0, 0, 0, 0, 0, 0];

m = multirotor(RotorPlacementAngles, RotorRotationDirections);
m.SetRotorAngles(RotorInwardAngle, RotorSidewardAngle, RotorDihedralAngle);
m.Visualize();
m.VisualizeAxes();
m.AnalyzeAccelerationManipulability(2);

c = controller(m);
tic
sim = simulation(m, c);

% sim.Controller.AttitudeController.SetPID(5, 0, 5);
% 
% pos = [0; 0; 0];
% vel = [0; 0; 0];
% rpy = [10; 20; 0];
% omega = [0; 0; 0];
% sim.Multirotor.SetInitialState(pos, vel, rpy, omega);
% sim.TotalTime = 10;
% sim.SimulateAttitudeResponse([-10; 0; 30], true);
% %sim.SimulatePositionResponse([1; 1; 1], 10, true);
% toc
% 
% graphics.PlotSignalsByName(sim, 3, {'pos', 'accel', 'rpy', 'rpy dot', 'rpm'}, true);
