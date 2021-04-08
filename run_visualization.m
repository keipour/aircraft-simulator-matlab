%% Prepare the environment

clear all
close all

addpath('simulator');

%% Define the hardware architecture
%r = robots.floating_hex();
%r = robots.octorotor_assymmetric();
%r = robots.quadrotor(true);
%r = robots.vtol_custom();
%r = robots.tilted_hex(true);
r = robots.odar(false);
%r = robots.odar_multilink(true);

%% Define the world

average_wind = [];
%w = worlds.empty_world(average_wind, true);
w = worlds.straight_wall(average_wind, false);
%w = worlds.sloped_wall_20_deg(average_wind, false);

%% Visualize

% Visualize the UAV
r.Visualize();
r.VisualizeAxes();

% Visualize the world
w.Visualize();
