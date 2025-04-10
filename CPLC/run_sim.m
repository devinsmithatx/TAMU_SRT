clear; clc; close all;       % create fresh session
addpath("initialization\");  % add initialization folder filepath
addpath("post_processing\"); % add post_processing folder filepath
addpath("simulink\");        % add simulink folder filepath

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INTRODUCTION & OVERVIEW

% run_sim.m takes in user input for hopper and runs an LQG simulation based 
% on user-defined parameters. The system is a rigid-body hopper, with up to
% 5 pairs of RCS thrusters and 1 single engine. 

%                  RCS THRUSTER PAIR LABELS                                         
%                                                                        
%       4   ________   5             2   ________   3                   
%       v  /        \  v             v  /        \  v                    
%    1 >o--|        |--o< 1       1 >o--|   cg   |--o< 1               
%       ^  |        |  ^             ^  \________/  ^                    
%       4  |   cg   |  5             2              3                    
%          |________|                                                    
%          / \____/ \                                                    
%         /          \                                                    
%              x                            z                            
%              ^                            ^                            
%              |__> y                       |__> y   

% The inner-loop controller is a Linear Quadratic Regulator (LQR), and 
% combined with a Kalman Filter, makes a Linear Quadratic Gaussian (LQG) 
% system. As of right now, guidance/trajectory modeling is simple step
% inputs with a PID controller. 

% A monte carlo analysis is run to get a region of stability for nominal 
% and off-nominal condtions.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% USER INPUTS - MISCELLANEOUS

% SIMULATION PARAMETERS
misc.T = 20;                % [s] simulation length
misc.type = "interactive";  % inner, outer, or interactive
misc.run_sim = true;        % run sim or go straight to plotting
misc.plot = true;           % generate plots

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% USER INPUTS - INITIAL CONDITIONS
% Define initial conditions for states and position

% INITIAL CONDITION (perturbation of x_bar)
ic.x_0 = [0.010;            % [rad]     roll initial condition
          0.011;            % [rad]     pitch initial condition  
          0.011;            % [rad]     yaw initial condition
          0.020;            % [m/s]     u initial condition
          0.050;            % [m/s]     v initial condition
          0.060;            % [m/s]     w initial condition
          0.001;            % [rad/s]   p initial condition
          0.001;            % [rad/s]   q initial condition 
          0.001;            % [rad/s]   r initial condition
          0.000;            % [N]       engine initial condition
          0.000;            % [N]       RCS pair initial condition
          0.000;            % [N]       RCS pair initial condition
          0.000;];          % [N]       RCS pair initial condition
ic.p0 =  [0.000;            % [m]       x position initial condition
          0.000;            % [m]       y position initial condition
          0.000;];          % [m]       z position initial condition

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% USER INPUTS - DYNAMICS MODELING 
% Define all the actuator and rigid body dynamics in this section.

% ACTUATOR DYNAMICS MODELING
act.rcs = [1 2 3];          %           Keep as [1 2 3] for now
act.tau_engine = 1/2;       % [Hz]      engine time response time
act.tau_rcs = 1/.03;        % [Hz]      rcs response time
act.lx = 0.25;              % [m]       thruster distance from C.G. along x
act.ly = 0.20;              % [m]       thruster distance from C.G. along y
act.rcs_max = 0.02224;      % [N]       max RCS thrust
act.engine_max = 22.4*1.25; % [N]       max engine thrust

% RIGID BODY DYNAMICS MODELING
body.m = 2.04117;           % [kg]      hopper mass
body.Ixx = 0.15952251;      % [kg*m^2]  Ixx moment of inertia
body.Iyy = 0.03961608;      % [kg*m^2]  Iyy moment of inertia
body.Izz = 0.12547452;      % [kg*m^2]  Izz moment of inertia
body.Ixy = 0.00654190;      % [kg*m^2]  Ixy moment of inertia
body.Ixz = 0.00578778;      % [kg*m^2]  Ixz moment of inertia
body.Iyz = 0.04033438;      % [kg*m^2]  Iyz moment of inertia
body.x_bar = [0.000;        % [rad]     roll trim point
              0.000;        % [rad]     pitch trim point  
              0.000;        % [rad]     yaw trim point
              0.000;        % [m/s]     u trim point
              0.000;        % [m/s]     v trim point
              0.000;        % [m/s]     w trim point
              0.000;        % [rad/s]   p trim point
              0.000;        % [rad/s]   q trim point 
              0.000;        % [rad/s]   r trim point
              9.810*body.m; % [N]       engine trim
              0.000;        % [N]       RCS pair trim 
              0.000;        % [N]       RCS pair trim 
              0.000;];      % [N]       RCS pair trim

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% USER INPUTS - GUIDANCE MODELING
% Where do you want to go? Define guidance sampling

guid.T = 1/40;               % [s]       guidance sampling

% TRAJECTORY COMMANDS (INNER / OUTER LOOP ONLY)
guid.p_ref = [5.000;         % [m]       x position command
              0.000;         % [m]       y position command
              0.000];        % [m]       z position command
guid.x_ref = [0.000;         % [rad]     roll command
              0.000;         % [rad]     pitch command  
              0.000;         % [rad]     yaw command
              5.000;         % [m/s]     u command
              0.000;         % [m/s]     v command
              0.000;         % [m/s]     w command
              0.000;         % [rad/s]   p command
              0.000;         % [rad/s]   q command 
              0.000;         % [rad/s]   r command
              0.000;         % [N]       engine command
              0.000;         % [N]       RCS pair command
              0.000;         % [N]       RCS pair command
              0.000;];       % [N]       RCS pair command

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% USER INPUTS - NAVIGATION MODELING & TUNING
% Define sensor sampling and available states, as well as noise and error
% parameters.

% SENSOR PARAMETERS
nav.T = 1/20;                     % [s] lowest sensor sampling period
nav.C = [eye(9) zeros(9,4)];      % available states for measureing

% NOISE CHARACTERISTICS
nav.Q = 0.001*eye(13);            % disturbance noise covariance matrix   
nav.R = 0.001*eye(9);             % measurement noise covariance matrix

% INITIAL KF PARAMETERS
nav.x0 = ic.x_0 + body.x_bar;     % initial esimate state
nav.P0 = 100*eye(13);             % initial error covariance

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% USER INPUTS - CONTROL MODELING & TUNING
% Define controller sampling as and LQR weights. Open lqr_gain and tune the
% weighting matrices inside the function. 

% CONTROLLER PARAMETERS
ctrl.T = 1/10;                            % [s] controller sampling period

% WEIGHT MATRICES
ctrl.R = diag([1/act.engine_max ones(1,3)/act.rcs_max]);
ctrl.Q                 = eye(13, 13);
ctrl.Q(1,1)            = 5;               % roll attitude
ctrl.Q(2:3,2:3)        = 20*eye(2);       % pitch and yaw atitude
ctrl.Q(4,4)            = 50;              % x velocity
ctrl.Q(5:6,5:6)        = 10*eye(2);       % y and z velocity
ctrl.Q(7,7)            = 1;               % roll rate
ctrl.Q(5:6,5:6)        = 5*eye(2);        % pitch and yaw rates
ctrl.Q(10:end, 10:end) = 0*ctrl.R;        % actuator states

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% USER INPUTS - OFF-NOMINAL CONDITTIONS & MONTE CARLO ANALYSIS
% not yet implemented

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% END USER INPUT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INITILIZATION

% ADDITONAL DYANMICS MODELING
act.u_bar = body.x_bar(10:end);                % actuator trim
[act.upper, act.lower] = actuator_limits(act); % actuator limits
body.I = inertia_tensor(body);                 % [kg*m^2] intertia tensor
[body.A, body.B] = state_space(body, act);     % linearization of dynamics

% ADDITIONAL NAVIGATION MODELING
[nav.phi, nav.gamma] = c2d(body.A, body.B, nav.T); % discetize dynamics

% ADDITIONAL CONTROL MODELING
ctrl.K = lqrd(body.A, body.B, ctrl.Q, ctrl.R, ctrl.T);  % controller gain

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SIMULINK

% RUN THE SIMULATION
if strcmp(misc.type, "inner")
    simout = sim("LQG_inner_loop");
elseif(strcmp(misc.type, "outer"))
    simout = sim("LQG_outer_loop");
elseif(strcmp(misc.type, "interactive"))
    open_system("LQG_outer_loop_interactive");
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% POST-PROCESSING

% PLOT DATA
if misc.plot
    plot_polezero(body, ctrl);       % plot pole-zero map of inner-loop
    plot_simout(simout, act, misc);  % plot time-series data
end