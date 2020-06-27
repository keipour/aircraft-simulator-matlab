clear all
close all

addpath('simulator');

RotorPlacementAngles = [30, 90, 150, 210, 270, 330];
RotorRotationDirections = [-1, 1, -1, 1, -1, 1];
RotorDihedralAngle = 0;
RotorSidewardAngle = [-30, 30, -30, 30, -30, 30];
RotorInwardAngle = 0;

m = multirotor.Create(RotorPlacementAngles, RotorRotationDirections);
m = multirotor.SetRotorAngles(m, RotorInwardAngle, RotorSidewardAngle, RotorDihedralAngle);

RotorSpeedsSquared = [55000, 55000, 55000, 55000, 55000, 55000];

tic
sim = simulation(m);
sim.Simulate(RotorSpeedsSquared);
toc

Pos = sim.GetStateTrajectory().GetPositions();

subplot(3, 1, 1);
plot(sim.GetTimeSteps(), Pos(:, 1))
subplot(3, 1, 2);
plot(sim.GetTimeSteps(), Pos(:, 2))
subplot(3, 1, 3);
plot(sim.GetTimeSteps(), Pos(:, 3))