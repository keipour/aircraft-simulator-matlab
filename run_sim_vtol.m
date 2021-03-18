%% Prepare the environment

clear all
close all

addpath('simulator');

%% Define the simulation

% Define the hardware architecture
m = robots.vtol_custom();

% Define the world
average_wind = [];
w = worlds.empty_world(average_wind, false);

% Define the controller
c = controllers.fully_actuated(m, attitude_strategies.FullTilt);

% Define the simulation object
sim = simulation(m, c, w);

%% Initial multirotor state

pos = [0; 0; -4];
vel = [0; 0; 0];
rpy = [0; 0; 0];
omega = [0; 0; 0];
sim.Multirotor.SetInitialState(pos, vel, rpy, omega);

%% Get the controller response(s)

% Simulate trajectory following
[traj, total_time] = trajectories.vtol_simple();
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
graphics.PlotSignalsByName(2, {'servo', 'inward', 'sideward'}, false, true);

%% Animate the result

fpv_cam = camera;
fpv_cam.Offset = [0; 0; -0.35];
graphics.AnimateLoggedTrajectory(sim.Multirotor, sim.Environment, 0, 1, true, true, []);
%graphics.RecordLoggedTrajectoryAnimation('myvideo', 30, sim.Multirotor, sim.Environment, 0, 1, true, true, fpv_cam);
