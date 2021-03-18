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
w = worlds.empty_world(average_wind, false);
%w = worlds.straight_wall(average_wind, false);
%w = worlds.sloped_wall_20_deg(average_wind, false);

% Define the controller
c = controllers.fully_actuated(r, attitude_strategies.FullTilt);

% Define the simulation object
sim = simulation(r, c, w);

%% Initial multirotor state

pos = [0; 0; -4];
vel = [0; 0; 0];
rpy = [0; 0; 0];
omega = [0; 0; 0];
sim.Multirotor.SetInitialState(pos, vel, rpy, omega);

%% Get the controller response(s)

% Simulate trajectory following
%[traj, total_time] = trajectories.copter_paint_air();
[traj, total_time] = trajectories.copter_two_points();
sim.SetTotalTime(total_time);
pos_thresh = 0.2;
rpy_thresh = 3; 
force_thresh = 0.2;
sim.SimulateTrajectory(traj, pos_thresh, rpy_thresh, force_thresh);

% Or simulate attitude response
%sim.SetTotalTime(10);
%figure; 
%sim.SimulateAttitudeResponse([0; 0; -90], true);

% Or simulate position response
%sim.SetTotalTime(10);
%figure;
%sim.SimulatePositionResponse([17; 8; -2], -45, true);

%% Draw Additional plots

graphics.PlotSignalsByName(3, {'pos', 'vel', 'accel', 'rpy', 'euler deriv', 'ang accel'}, false, true);

%% Animate the result

fpv_cam = camera;
fpv_cam.Offset = [0; 0; -0.35];
graphics.AnimateLoggedTrajectory(sim.Multirotor, sim.Environment, 0, 1, true, true, []);
%graphics.RecordLoggedTrajectoryAnimation('myvideo', 30, sim.Multirotor, sim.Environment, 0, 1, true, true, fpv_cam);
