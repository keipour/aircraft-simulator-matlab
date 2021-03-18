%% Prepare the environment

clear all
close all

addpath('simulator');

%% Define the hardware architecture
m = robots.vtol_custom();

%% Define the world

average_wind = [];
w = worlds.empty_world(average_wind, false);
%w = worlds.straight_wall(average_wind, false);
%w = worlds.sloped_wall_20_deg(average_wind, false);

%% Define the controller

c = controllers.fully_actuated(m, attitude_strategies.FullTilt);

%% Prepare the simulation

sim = simulation(m, c, w);

sim.SetTotalTime(15);

%% Initial multirotor state

pos = [0; 0; -4];
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

% Trajectory for VTOL:
traj = {
    [2, 0, -4, 0], [0, 0]; 
    [20, 0, -4, 0], [90, 90]; 
    };

sim.SimulateTrajectory(traj, 0.2, 3, 0.2);

%% Draw Additional plots

graphics.PlotSignalsByName(3, {'pos', 'vel', 'accel', 'rpy', 'euler deriv', 'ang accel'}, false, true);
graphics.PlotSignalsByName(2, {'servo', 'inward', 'sideward'}, false, true);

%% Animate the result

fpv_cam = camera;
fpv_cam.Offset = [0; 0; -0.35];
graphics.AnimateLoggedTrajectory(sim.Multirotor, sim.Environment, 0, 1, true, true, []);
%graphics.RecordLoggedTrajectoryAnimation('myvideo', 30, sim.Multirotor, sim.Environment, 0, 1, true, true, fpv_cam);
