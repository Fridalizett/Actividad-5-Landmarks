%% EXAMPLE: Differential drive vehicle following waypoints using the 
% Pure Pursuit algorithm
%
% Copyright 2018-2019 The MathWorks, Inc.
clear all
clc

%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.1;               % Sample time [s]
tVec = 0:sampleTime:80;         % Time array

initPose = [2;2;pi];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

% Define waypoints
%waypoints = [0,0; 2,2; 4,2; 2,4; 0.5,3];

%waypoints1 = [2,2; -5,4; 4,-0.5; -3.5,-3; 0,2.5; -1.5,0; 1,-2.5; 0,0];

%waypoints2 = [2,5; -5,3; -5,-2; 2,-5; 5,2; -3,2; -4,-4; 4,-3];

%waypoints3 = [-3,4; 3,3; 1,-3; -1,-1; 1,4; -3,-4; 2,-1];

%perro
waypoints = [-4,3; -5,2; -5,3; -3,3; -3,4; -1,4; -2,3; -2,4.3; 0,5; -1,6; -1,4.8; 0,5; 1,5; 1,4; 1,6; 2,3; 1,2; 2,3; 1,6; 3,2; 5,0; 0,-2; 0,-1; 4,1; 0,-1; -1,0; -4,0; -3,1; -2,1; -3,1; -4,0; -5,2];

%flor
%waypoints = [4,1; 2,1; 0,3; 2,3; 4,1; 6,1; 8,3; 6,3; 4,1; 4,5; 2,3; 2,5; 0,5; 2,7; 0,9; 2,9; 2,11; 4,9; 6,11; 6,9; 8,9; 6,7; 8,5; 6,5; 6,3; 3,6; 3,8; 5,8; 5,6; 3,6];

%cereza
%waypoints = [5,1; 4,2; 3,3; 3,4; 3,5; 4,6; 5,7; 6,7; 7,7 ; 8,6; 7,5; 6,6; 7,5; 8,5; 7,5; 8,6; 10,9; 9,10; 8,11; 6,11; 5,10; 4,9; 10,9; 8,6; 9,5; 9,3; 8,2; 7,1; 5,1];


% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.35;
controller.DesiredLinearVelocity = 0.75;
controller.MaxAngularVelocity = 1.5;

%% Simulation loop
close all
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef);
    
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints)
    waitfor(r);
end
