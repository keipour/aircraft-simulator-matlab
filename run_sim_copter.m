%% Prepare the environment

clear all
close all

addpath('simulator');

%% Define the simulation

% Define the hardware architecture
%r = robots.floating_hex();
%r = robots.octorotor_assymmetric();
%r = robots.quadrotor();
r = robots.tilted_hex(true);
%r = robots.odar();

% Define the world
average_wind = [];
%w = worlds.empty_world(average_wind, false);
w = worlds.straight_wall(average_wind, false);
%wall_angle = -20; w = worlds.sloped_wall(wall_angle, average_wind, false);

% Define the controller
c = controllers.fully_actuated(r, attitude_strategies.ZeroTilt);

% Define the simulation object
sim = simulation(r, c, w);

%% Simulate the desired experiment with trajectory

%[pos, vel, rpy, omega, traj, total_time] = experiments.copter_paint_air();
[pos, vel, rpy, omega, traj, total_time] = experiments.copter_two_points();
%[pos, vel, rpy, omega, traj, total_time] = experiments.copter_touch_wall();
sim.SetTotalTime(total_time);

sim.Multirotor.SetInitialState(pos, vel, rpy, omega);

pos_thresh = 0.2;
rpy_thresh = 3; 
force_thresh = 0.2;
sim.SimulateTrajectory(traj, pos_thresh, rpy_thresh, force_thresh);

%% Or get the controller response(s)

% Or simulate attitude response
%sim.SetTotalTime(10);
%figure; 
%sim.SimulateAttitudeResponse([0; 0; -90], true);

% Or simulate position response
%sim.SetTotalTime(10);
%figure;
%sim.SimulatePositionResponse([17; 8; -2], -45, true);

%% Draw Additional plots

graphics.PlotSignalsByName(1, {'pos', 'rpy'}, false, true);

%% Animate the result

fpv_cam = camera;
fpv_cam.Offset = [0; 0; -0.35];
graphics.AnimateLoggedTrajectory(sim.Multirotor, sim.Environment, 0, 1, true, true, []);
%graphics.RecordLoggedTrajectoryAnimation('myvideo', 30, sim.Multirotor, sim.Environment, 0, 1, true, true, fpv_cam);
