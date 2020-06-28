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

c = controller(m);
c.AttitudeController.SetPID(5*eye(3), 0*eye(3), 5*eye(3));

tic
sim = simulation(m, c);
sim.TotalTime = 30;
sim.Simulate();
toc

Pos = sim.GetStateTrajectory().GetPositions();
RPY = sim.GetStateTrajectory().GetRPYs();
Fs = sim.GetStateTrajectory().GetAccelerations();
Ms = sim.GetStateTrajectory().GetAngularAccelerations();
t = sim.GetTimeSteps();

subplot(3, 4, 1);
spplot(t, Pos(:, 1), 'x')
subplot(3, 4, 5);
spplot(t, Pos(:, 2), 'y')
subplot(3, 4, 9);
spplot(t, Pos(:, 3), 'z')

subplot(3, 4, 2);
spplot(t, RPY(:, 1))
subplot(3, 4, 6);
spplot(t, RPY(:, 2))
subplot(3, 4, 10);
spplot(t, RPY(:, 3))

subplot(3, 4, 3);
spplot(t, Fs(:, 1))
subplot(3, 4, 7);
spplot(t, Fs(:, 2))
subplot(3, 4, 11);
spplot(t, Fs(:, 3))

subplot(3, 4, 4);
spplot(t, Ms(:, 1))
subplot(3, 4, 8);
spplot(t, Ms(:, 2))
subplot(3, 4, 12);
spplot(t, Ms(:, 3))

ca = control_allocation(m);
ca.CalcRotorSpeeds(m, [0; 0; 0], [0; 0; 2])

function spplot(t, X, str)
    plot(t, X);
    lim = ylim;
    ylim([lim(1) - 1, lim(2) + 1]);
    if nargin > 2
        title(str);
    end
end