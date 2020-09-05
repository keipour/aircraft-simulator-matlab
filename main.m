%% Prepare the environment

clear all
close all
try
    delete(findall(0));
catch
end

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
e.AddCuboidObject([15; 10; -2.5], [1; 10; 5], 0);
e.AddGroundPlane([-100; 100], [-100; 100]);

%% Prepare the simulation

sim = simulation(m, controller(m), e);

% Initial multirotor state
pos = [10; 10; -2];
vel = [0; 0; 0];
rpy = [0; 0; 0];
omega = [0; 0; 0];
sim.Multirotor.SetInitialState(pos, vel, rpy, omega);

sim.SetTotalTime(20);

%% Prepare the controller

sim.Controller.AttitudeController.SetPID(60, 0, 20);
sim.Controller.PositionController.SetPID(3, 0, 7);
sim.Controller.PositionController.AttitudeType = attitude_types.ZeroTilt;

%% Get the controller response(s)

% Attitude response
%figure; 
%sim.SimulateAttitudeResponse([0; 0; -90], true);

% Position response
figure;
sim.SimulatePositionResponse([17; 8; -2], 0, true);

% Trajectory following
%traj = [12, 12, -4, 0; 12, 16, -3, 90];
%sim.SimulateTrajectory(traj, 0.25);

% Additional plots
graphics.PlotSignalsByName(3, {'pos', 'vel', 'accel', 'rpy', 'euler deriv', 'ang accel'}, true);

%% Animate the result

graphics.AnimateLoggedTrajectory(sim.Multirotor, sim.Environment, 0, 1, true, true);
%graphics.RecordLoggedTrajectoryAnimation('myvideo', 30, sim.Multirotor, sim.Environment, 0, 1, true, true);
