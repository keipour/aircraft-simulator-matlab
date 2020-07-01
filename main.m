clear all
close all

addpath('simulator');

RotorPlacementAngles = [30, 90, 150, 210, 270, 330];
RotorRotationDirections = [-1, 1, -1, 1, -1, 1];
RotorDihedralAngle = 0;
RotorSidewardAngle = [-30, 30, -30, 30, -30, 30];
RotorInwardAngle = 0;

m = multirotor(RotorPlacementAngles, RotorRotationDirections);
m.SetRotorAngles(RotorInwardAngle, RotorSidewardAngle, RotorDihedralAngle);
pos = [0; 0; 0];
vel = [0; 0; 0];
rpy = [0; 0; 0];
omega = [0; 0; 0];

c = controller(m);

tic
sim = simulation(m, c);


sim.Controller.AttitudeController.SetPID(5, 0, 5);
sim.Multirotor.SetInitialState(pos, vel, rpy, omega);
sim.TotalTime = 10;
sim.SimulateAttitudeResponse([10; -10; 50], true);
toc

t = sim.GetTimeSteps();
graphics.PlotSignalsByName(sim, 3, {'pos', 'accel', 'rpy', 'rpy dot', 'rpm'}, true);
