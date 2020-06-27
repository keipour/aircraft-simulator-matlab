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
m.SetInitialState(pos, vel, rpy, omega);


RotorSpeedsSquared = [55000, 55000, 55000, 55000, 55000, 55000];

tic
sim = simulation(m);
sim.TotalTime = 5;
sim.Simulate(RotorSpeedsSquared);
toc

Pos = sim.GetStateTrajectory().GetPositions();
RPY = sim.GetStateTrajectory().GetRPYs();
Fs = sim.GetStateTrajectory().GetForces();
Ms = sim.GetStateTrajectory().GetMoments();
t = sim.GetTimeSteps();

subplot(3, 4, 1);
plot(t, Pos(:, 1))
subplot(3, 4, 5);
plot(t, Pos(:, 2))
subplot(3, 4, 9);
plot(t, Pos(:, 3))

subplot(3, 4, 2);
plot(t, RPY(:, 1))
subplot(3, 4, 6);
plot(t, RPY(:, 2))
subplot(3, 4, 10);
plot(t, RPY(:, 3))

subplot(3, 4, 3);
plot(t, Fs(:, 1))
subplot(3, 4, 7);
plot(t, Fs(:, 2))
subplot(3, 4, 11);
plot(t, Fs(:, 3))

subplot(3, 4, 4);
plot(t, Ms(:, 1))
subplot(3, 4, 8);
plot(t, Ms(:, 2))
subplot(3, 4, 12);
plot(t, Ms(:, 3))
