%% Prepare the environment

clear all
close all

addpath('simulator');

%% Define the hardware architecture
%m = robots.floating_hex();
%m = robots.octorotor_assymmetric();
%m = robots.quadrotor(true);
m = robots.vtol();
%m = robots.tilted_hex(true);

%% Define the world

average_wind = [];
%w = worlds.empty_world(average_wind, true);
w = worlds.straight_wall(average_wind, false);
%w = worlds.sloped_wall_20_deg(average_wind, false);

%% Visualize

% Visualize the UAV
m.Visualize();
m.VisualizeAxes();

% Visualize the world
w.Visualize();
