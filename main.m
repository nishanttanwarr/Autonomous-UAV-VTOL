%% Autonomous VTOL UAV Landing on Moving Vehicle Simulation
% This simulation demonstrates a drone tracking and landing on one of three
% moving vehicles with ArUco marker authentication in a 3D environment.
%
% Features:
% - Professional dashboard with real-time telemetry and visualization
% - Three vehicles moving in a convoy with different ArUco marker IDs
% - Drone with autonomous navigation and tracking capabilities
% - ArUco marker detection and authentication for secure landing
% - Multiple camera views (drone POV, third-person, and top-down)
% - Manual and autonomous control modes
% - Dynamic adaptation to environmental constraints

clear all;
close all;
clc;

% Add all subdirectories to path
addpath(genpath('./'));

% Initialize simulation
sim = DroneSimulation();

% Run the simulation
sim.run();
