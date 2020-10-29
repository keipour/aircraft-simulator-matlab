%% Prepare the environment

clear all
close all

addpath('simulator');

%% Define the hardware architecture
%m = robots.floating_hex();
%m = robots.octorotor_assymmetric();
%m = robots.quadrotor(true);
m = robots.tilted_hex(true);

%% Define the world

average_wind = [];
%w = worlds.empty_world(average_wind, true);
w = worlds.straight_wall(average_wind, false);
%w = worlds.sloped_wall_20_deg(average_wind, false);

%% Define the controller

c = controllers.fully_actuated(m, attitude_strategies.ZeroTilt);

%% Prepare the simulation

sim = simulation(m, c, w);

sim.SetTotalTime(10);
%sim.SetTotalTime(75); % For the AIR trajectory

%% Initial multirotor state

pos = [10; 10; -4];
vel = [0; 0; 0];
rpy = [0; 0; 0];
omega = [0; 0; 0];
sim.Multirotor.SetInitialState(pos, vel, rpy, omega);

%% Get the controller response(s)

% Attitude response
%figure; 
%sim.SimulateAttitudeResponse([0; 0; -90], true);

% Position response
%figure;
%sim.SimulatePositionResponse([17; 8; -2], -45, true);

% Trajectory following
traj = [12, 12, -4, 0; 12, 16, -3, 90];
% traj = {[13, 6, -1, 0];
%         [13.25, 6, -1, 0]; 
%         [13.25, 6, -1, 0, 5, 0, 0];
%         [13.25, 7, -4, 0, 5, 0, 0];
%         [13.25, 8, -1, 0, 5, 0, 0]
%         [13.25, 7.5, -2.1, 0, 5, 0, 0]
%         [13.25, 6.2, -2.1, 0, 5, 0, 0]
%         [12.8, 6.2, -2.1, 0];
%         [12.8, 9.5, -4, 0];
%         [13.25, 9.5, -4, 0, 5, 0, 0]; 
%         [13.25, 9.5, -1, 0, 5, 0, 0];
%         [12.8, 9.5, -1, 0];
%         [12.8, 11.5, -1, 0];
%         [13.25, 11.5, -1, 0, 5, 0, 0];
%         [13.25, 11.5, -4, 0, 5, 0, 0];
%         [13.25, 12, -4, 0, 5, 0, 0];
%         [13.25, 12.5, -3.5, 0, 5, 0, 0];
%         [13.25, 12.5, -3, 0, 5, 0, 0];
%         [13.25, 12, -2.5, 0, 5, 0, 0];
%         [13.25, 11.5, -2.5, 0, 5, 0, 0];
%         [13.25, 13, -1, 0, 5, 0, 0];
%         [12.8, 13, -1, 0];
%         [10, 10, -4, 0];
%         };
sim.SimulateTrajectory(traj, 0.2, 3, 0.2);

%% Draw Additional plots

graphics.PlotSignalsByName(3, {'pos', 'vel', 'accel', 'rpy', 'euler deriv', 'ang accel', 'wind'}, true);

%% Animate the result

fpv_cam = camera;
fpv_cam.Offset = [0; 0; -0.35];
graphics.AnimateLoggedTrajectory(sim.Multirotor, sim.Environment, 0, 1, true, true, []);
%graphics.RecordLoggedTrajectoryAnimation('myvideo', 30, sim.Multirotor, sim.Environment, 0, 1, true, true, fpv_cam);
