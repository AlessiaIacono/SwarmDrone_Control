%% PARAMETERS.M (multi-scenario, comment/uncomment)
% This script initializes ALL the workspace variables required by the Simulink model.
% Usage:
%   1) Uncomment ONE scenario in the "SCENARIO SELECTION" section
%   2) Run this file
%   3) Run the Simulink model


clear; clc; close all;

%% =========================
%  0) SCENARIO SELECTION
% =========================
% Uncomment exactly ONE line:

scenario_id = 'A';   % Case Study A: Balanced Performance (obstacle wall)
%scenario_id = 'B';  % Case Study B: Aggressive Maneuver (stress test)
%scenario_id = 'C';  % Case Study C: Symmetric Deadlock / Connectivity Trap
%scenario_id = 'C2'; % Case Study C2: Deadlock Resolution via Relaxed Connectivity

disp(['Loading scenario: ', scenario_id]);

%% =========================
%  1) COMMON SYSTEM PARAMETERS
% =========================
n_robots = 4;

% Sampling time used by the discrete planner/controller blocks.
Ts = 0.01;

% Abort / termination logic
t_sim = 100.0;          % watchdog timeout
goal_tolerance = 0.05;  % [m]

% Physical parameters (Crazyflie 2.1)
mass = 0.032;   % [kg]
g = 9.81;    % [m/s^2]
l = 0.046;   % [m] arm length
Jxx = 1.4e-5; Jyy = 1.4e-5; Jzz = 2.2e-5;
Ib = diag([Jxx, Jyy, Jzz]);

% Actuation limits
max_acc   = 2.5;     % [m/s^2] planner acceleration bound (u)
tau_max   = 0.003;   % [N*m] attitude torque saturation (inner loop)


% Safety / CBF parameters (HOCBF: h_ddot + gamma1*h_dot + gamma2*h >= 0)
gamma1 = 8.0;
gamma2 = 8.0;

% Robot geometry + safety margin
radius_robots = 0.05; % [m]
margin = 0.10; % [m]
Rs_safe = radius_robots + margin; % [m] minimum clearance radius (robot-obstacle and robot-robot)
Rs = Rs_safe;

% Connectivity graph (Ring): 1-2-3-4-1
Adj_mat = [0 1 0 1;
           1 0 1 0;
           0 1 0 1;
           1 0 1 0];
adj_row_1 = Adj_mat(1,:); adj_row_2 = Adj_mat(2,:);
adj_row_3 = Adj_mat(3,:); adj_row_4 = Adj_mat(4,:);

%% =========================
%  2) SCENARIO-SPECIFIC SETTINGS
% =========================
switch scenario_id

    case 'A'
        %% CASE A: Balanced Performance 
        Kp = 1.2;
        Kd = 2.5;
        Rc = 1.0;

        % Obstacles
        x_objects = [ -0.4,   0.0,    0.4,   -0.2,   0.2;     % X
               1.2,   1.2,    1.2,    1.8,   1.8;             % Y
               0.5,   0.6,    0.5,    0.9,   0.9];            % Z

        radii_obstacles = [0.30, 0.30, 0.30, 0.35, 0.35];

        n_obs = size(x_objects, 2);
        Rs_obs = radius_robots + radii_obstacles + margin;

        % Initial: square on ground
        x0 = [ -0.5, -0.5,  0.0;   % R1
               -0.5,  0.5,  0.0;   % R2
                0.5,  0.5,  0.0;   % R3
                0.5, -0.5,  0.0];  % R4
        v0 = zeros(4, 3);

        % Goal: "Diamond" at z = 1.5
        xg = [ -0.5,  3.5,  1.5;   % R1
               -0.2,  3.5,  1.5;   % R2 
                0.2,  3.5,  1.5;   % R3
                0.5,  3.5,  1.5];  % R4
        xg_vec = reshape(xg', [], 1);

        desired_flight_time = 10.0; 
        dist_total = norm(xg(1,:) - x0(1,:));
        max_vel = 1.2 * (dist_total / desired_flight_time); % pacing rule

    case 'B'
        %% CASE B: Aggressive Maneuver 
        Kp = 3.0;
        Kd = 0.8;
        Rc = 1.0;

        % Same environment as Case A (wall of obstacles)
        x_objects = [ -0.4,   0.0,    0.4,   -0.2,   0.2;    
               1.2,   1.2,    1.2,    1.8,   1.8;   
               0.5,   0.6,    0.5,    0.9,   0.9];

        radii_obstacles = [0.30, 0.30, 0.30, 0.35, 0.35];

        n_obs = size(x_objects, 2);
        Rs_obs = radius_robots + radii_obstacles + margin;

        % Initial: square on ground
        x0 = [ -0.5, -0.5,  0.0;   % R1
               -0.5,  0.5,  0.0;   % R2
                0.5,  0.5,  0.0;   % R3
                0.5, -0.5,  0.0];  % R4
        v0 = zeros(4, 3);

        % Goal: "Diamond" at z = 1.5
        xg = [ -0.5,  3.5,  1.5;   % R1
               -0.2,  3.5,  1.5;   % R2 
                0.2,  3.5,  1.5;   % R3
                0.5,  3.5,  1.5];  % R4
        xg_vec = reshape(xg', [], 1);

      
        desired_flight_time = 10.0;
        dist_total = norm(xg(1,:) - x0(1,:));
        max_vel = 1.2 * (dist_total / desired_flight_time);

    case 'C'
        %% CASE C: Symmetric Deadlock / Connectivity Trap
        Kp = 1.2;
        Kd = 2.5;

        Rc = 0.8; % tight connectivity -> causes deadlock with large obstacle

        % Single large obstacle on the path 
        x_objects = [0.0;
                     1.8;
                     0.7];
        radii_obstacles = 0.8;   
        n_obs = size(x_objects, 2);
        Rs_obs = radius_robots + radii_obstacles + margin;

        % Symmetric start (tight square around x=0)
        x0 = [ -0.2, -0.2, 0.0;
               -0.2,  0.2, 0.0;
                0.2,  0.2, 0.0;
                0.2, -0.2, 0.0 ];
        v0 = zeros(4, 3);

        % Symmetric goals behind the obstacle
        xg = [ -0.2,  3.5, 1.0;
               -0.2,  3.9, 1.0;
                0.2,  3.9, 1.0;
                0.2,  3.5, 1.0 ];
        xg_vec = reshape(xg', [], 1);

        desired_flight_time = 15.0;
        dist_total = norm(xg(1,:) - x0(1,:));
        max_vel = 1.2 * (dist_total / desired_flight_time);

    case 'C2'
        %% CASE C2: Deadlock Resolution via Relaxed Connectivity
        Kp = 1.2;
        Kd = 2.5;

        Rc = 2.0; % relaxed connectivity allows splitting around obstacle

        % Same trap obstacle
        x_objects = [0.0;
                     1.8;
                     0.7];
        radii_obstacles = 0.8;
        n_obs = size(x_objects, 2);
        Rs_obs = radius_robots + radii_obstacles + margin;

        % Same start/goal as Case C
        x0 = [ -0.2, -0.2, 0.0;
               -0.2,  0.2, 0.0;
                0.2,  0.2, 0.0;
                0.2, -0.2, 0.0 ];
        v0 = zeros(4, 3);

        xg = [ -0.2,  3.5, 1.0;
               -0.2,  3.9, 1.0;
                0.2,  3.9, 1.0;
                0.2,  3.5, 1.0 ];
        xg_vec = reshape(xg', [], 1);

        desired_flight_time = 15.0;
        dist_total = norm(xg(1,:) - x0(1,:));
        max_vel = 1.2 * (dist_total / desired_flight_time);

    otherwise
        error('Unknown scenario_id. Use ''A'', ''B'', ''C'', or ''C2''.');
end

%% =========================
%  3) SIMULINK-CONVENIENCE VECTORS
% =========================
% Convert to column vectors as expected by the blocks
x0_1 = x0(1,:)'; v0_1 = v0(1,:)'; xg_1 = xg(1,:)';
x0_2 = x0(2,:)'; v0_2 = v0(2,:)'; xg_2 = xg(2,:)';
x0_3 = x0(3,:)'; v0_3 = v0(3,:)'; xg_3 = xg(3,:)';
x0_4 = x0(4,:)'; v0_4 = v0(4,:)'; xg_4 = xg(4,:)';

disp('Parameters loaded successfully.');
disp(['Kp=', num2str(Kp), ', Kd=', num2str(Kd), ', Rc=', num2str(Rc), ', max_vel=', num2str(max_vel)]);
