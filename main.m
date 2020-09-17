%% Prepare the environment

clear all
close all

addpath('simulator');

%% Define the hardware architecture
%m = robots.floating_hex();
%m = robots.quadrotor();
m = robots.tilted_hex(true);

%m.Visualize();
%m.VisualizeAxes();
%m.AnalyzeDynamicManipulability(2, 2);

%% Define the environment

% Add a wall and the ground to the environment
e = environment;
e.AddCuboidObject([15; 10; -2.5], [2; 10; 5], 0);
e.AddGroundPlane([-100; 100], [-100; 100]);
e.AverageWind = [0; 0; 0];

%% Prepare the simulation

sim = simulation(m, controller(m), e);

sim.SetTotalTime(20);

%% Prepare the controller

sim.Controller.AttitudeController.SetPID(60, 0, 20);
sim.Controller.PositionController.SetPID(3, 0, 7);
sim.Controller.HMFCController.ForceController.SetPID(1, 0, 3);
sim.Controller.HMFCController.PositionController.SetPID(3, 0, 7);
sim.Controller.SetAttitudeStrategy(attitude_strategies.ZeroTilt);

%% Initial multirotor state

pos = [10; 10; -2];
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
%traj = [12, 12, -4, 0; 12, 16, -3, 90];
traj = [17, 8, -2, 0];
sim.SimulateTrajectory(traj, 0.25);

%% Draw Additional plots
%graphics.PlotSignalsByName(3, {'pos', 'vel', 'accel', 'rpy', 'euler deriv', 'ang accel', 'wind'}, true);

%% Animate the result

fpv_cam = camera;
fpv_cam.Offset = [-0.35; 0; -0.25];
graphics.AnimateLoggedTrajectory(sim.Multirotor, sim.Environment, 0, 1, true, true, []);
%graphics.RecordLoggedTrajectoryAnimation('myvideo', 30, sim.Multirotor, sim.Environment, 0, 1, true, true, fpv_cam);
