clc; clear all; close all;
%% Parameters
student_ID = 11908203;
m = 4;      % Mass [kg]
J = 0.0475; % Moment of inertia [kg.m^2]
r = 0.25;   % Thrust offset [m]
g = 9.81;   % Gravity [m/s^2]
c = 0.05;   % Damping [N.s/m]

%% Equilibrium Points
u1_bar = 0; u2_bar = 0;
x_ref = 3; % Refrence point    
y_ref = 3; % Refrence point  
x_bar = [m*x_ref; 0; m*y_ref; 0; 0; 0];  % scaled: [4*3;0;4*3;0;0;0] target(12, 12)

% Symbolic Variables 
syms x1 x2 x3 x4 x5 x6 u1 u2 real

% Equations for NonLinear system
F1 = u1;
F2 = u2 + m*g;
x_dot_1 = x2;
x_dot_3 = x4;
x_dot_5 = x6;
x_dot_2 = (1/m)*(F1*cos(x5) - F2*sin(x5) - c*x2);
x_dot_4 = (1/m)*(F1*sin(x5) + F2*cos(x5) - c*x4) - g;
x_dot_6 = (1/J)*(r*F1);

%% Linearisation
% Symbolic jacobian matrices
A = jacobian([x_dot_1, x_dot_2, x_dot_3, x_dot_4, x_dot_5, x_dot_6], ...
    [x1, x2, x3, x4, x5, x6]);
B = jacobian([x_dot_1, x_dot_2, x_dot_3, x_dot_4, x_dot_5, x_dot_6], [u1, u2]);

Ce = [1,0,0,0,0,0;   % measurable outputs: x position, y position, angle
      0,0,1,0,0,0;
      0,0,0,0,1,0];
De = zeros(3,2);

% Evaluate at equilibrium (Physical system, for obserser design later)
Ae_phys = double(subs(A, [x1;x2;x3;x4;x5;x6;u1;u2], [x_bar;u1_bar;u2_bar]));
Be_phys = double(subs(B, [x1;x2;x3;x4;x5;x6;u1;u2], [x_bar;u1_bar;u2_bar]));

% Scale for black box coordinate system (For controller design)
S     = diag([m, m, m, m, J, J]);
S_inv = diag([1/m, 1/m, 1/m, 1/m, 1/J, 1/J]);
Ae    = S * Ae_phys * S_inv;   % scaled version
Be    = S * Be_phys;           % scaled version

% Linearised State Space Model
VTOL_Linearised = ss(Ae, Be, Ce, De);

%% Linearisation Analysis - Initial conditions
Ts = 0.01;                                             % Sampling time
t  = 0:Ts:10;                                          % Time vector - 10 seconds
ut = zeros(length(t), 2);                              % Zero input (2 inputs)
x0    = x_bar + [m*(-x_ref); 0; m*(-y_ref); 0; 0; 0];  % Initial condition start at origin (0,0)
delta_x0 = x0 - x_bar;

% Full state output for linearisation validation ONLY
VTOL_Linearised_full = ss(Ae, Be, eye(6), zeros(6,2));

% Simulate full state linearised model
ylin = lsim(VTOL_Linearised_full, ut, t, delta_x0);

% all 6 states directly from linear model
x1_lin     = ylin(:,1) + x_bar(1);    % X position
x2_lin     = ylin(:,2);               % X velocity 
x3_lin     = ylin(:,3) + x_bar(3);    % Y position
x4_lin     = ylin(:,4);               % Y velocity 
x5_lin_deg = ylin(:,5) * (180/pi);    % Angle
x6_lin     = ylin(:,6) * (180/pi);    % Angular rate 

%% Non linear system
F_input = @(t, x) [0; m*g];   % hover input

% Simulate Nonlinear models
[t_nl,   x_nl]   = ode45(@(t,x) VTOL_Dynamics(t,x,F_input,0),t, x0);            % WITHOUT disturbance

[t_nl_d, x_nl_d] = ode45(@(t,x) VTOL_Dynamics(t,x,F_input,student_ID), t, x0);  % WITH disturbance

% Nonlinear states — WITHOUT disturbance
x1_nl     = x_nl(:,1);
x2_nl     = x_nl(:,2);
x3_nl     = x_nl(:,3);
x4_nl     = x_nl(:,4);
x5_nl_deg = x_nl(:,5) * (180/pi);
x6_nl     = x_nl(:,6) * (180/pi);

% Nonlinear states — WITH disturbance
x1_nl_d     = x_nl_d(:,1);
x2_nl_d     = x_nl_d(:,2);
x3_nl_d     = x_nl_d(:,3);
x4_nl_d     = x_nl_d(:,4);
x5_nl_d_deg = x_nl_d(:,5) * (180/pi);
x6_nl_d     = x_nl_d(:,6) * (180/pi);

% Plotting Linear vs Nonlinear (Shows Linearised Correctly)
figure;
sgtitle('Linear vs Nonlinear Model Comparison')
subplot(3,2,1);
plot(t, x1_lin, 'b', t_nl, x1_nl, 'r--', t_nl_d, x1_nl_d, 'g--');
ylim([-1, 10]);
ylabel('X Position (m)'); xlabel('Time (s)');
legend('Linear','Nonlinear','Nonlinear (Dist)'); title('Horizontal Position');
subplot(3,2,2);
plot(t, x2_lin, 'b', t_nl, x2_nl, 'r--', t_nl_d, x2_nl_d, 'g--');
ylim([-1, 3]);
ylabel('X Velocity (m/s)'); xlabel('Time (s)');
legend('Linear','Nonlinear','Nonlinear (Dist)'); title('Horizontal Velocity');
subplot(3,2,3);
plot(t, x3_lin, 'b', t_nl, x3_nl, 'r--', t_nl_d, x3_nl_d, 'g--');
ylabel('Altitude (m)'); xlabel('Time (s)');
legend('Linear','Nonlinear','Nonlinear (Dist)'); title('Altitude');
subplot(3,2,4);
plot(t, x4_lin, 'b', t_nl, x4_nl, 'r--', t_nl_d, x4_nl_d, 'g--');
ylabel('Y Velocity (m/s)'); xlabel('Time (s)');
legend('Linear','Nonlinear','Nonlinear (Dist)'); title('Vertical Velocity');
subplot(3,2,5);
plot(t, x5_lin_deg, 'b', t_nl, x5_nl_deg, 'r--', t_nl_d, x5_nl_d_deg, 'g--');
ylabel('Angle (deg)'); xlabel('Time (s)');
legend('Linear','Nonlinear','Nonlinear (Dist)'); title('Angle');
subplot(3,2,6);
plot(t, x6_lin, 'b', t_nl, x6_nl, 'r--', t_nl_d, x6_nl_d, 'g--');
ylabel('Angular Rate (deg/s)'); xlabel('Time (s)');
legend('Linear','Nonlinear','Nonlinear (Dist)'); title('Angular Rate');
saveas(gcf,'LinearVsNonlinear.png');

%% Open loop Stability Analysis
% Discretisation (Controller system, scaled)
sysD = c2d(VTOL_Linearised, Ts, 'zoh');
Gd = sysD.A; Hd = sysD.B; C = sysD.C; D = sysD.D; 

% Discretise (Observer system, Physcial)
sysD_phys = c2d(ss(Ae_phys, Be_phys, Ce, De), Ts, 'zoh');
Gd_phys   = sysD_phys.A; Hd_phys   = sysD_phys.B;

% Print open loop poles & stabilty check for report
printPoles(Gd, Hd, [],  'Open Loop')

% Plot open loop poles (Show Linearized Discrete System Stability)
figure;
sgtitle('Linearised System, Open loop Pole-Zero Map');
% Full unit circle subplot
ax1 = subplot(1,2,1);
pzplot(ax1, sysD);
title('Full Unit Circle')
ax1.XLim = [-1.2, 1.2]; ax1.YLim = [-1.2, 1.2]; % axis control
% Zoomed in subplot
ax2 = subplot(1,2,2);
pzplot(ax2, sysD);
title("Zoomed In Pole locations")
ax2.XLim = [0, 1.2]; ax2.YLim = [-0.3, 0.3]; % axis control   
saveas(gcf, 'OLsystemPoleMap.png');

%% Optimal Controller Design
% Controlability Check
fprintf('\n') % Add space in command window
if rank(ctrb(Gd,Hd))==size(Gd,1)
    disp('System is controllable')
else
    disp('System is not controllable')
end
fprintf('\n') % Add space in command window

% Q and R tuning for Kd 
q1 = 200;   % X position         
q2 = 300;   % X velocity
q3 = 50;    % Y position (Altitude)
q4 = 50;    % Y velocity
q5 = 400;   % Roll Angle (Crucial for stability)
q6 = 250;   % Angular Rate

Q6 = diag([q1, q2, q3, q4, q5, q6]);
R6 = eye(2)*5;  

[Kd, ~, DesiredPoles] = dlqr(Gd, Hd, Q6, R6); % Extract gain and desired pole locations

% Output selection matrix 
Cr = [1, 0, 0, 0, 0, 0;   % selects state 1 — X position
      0, 0, 1, 0, 0, 0];  % selects state 3 — Y position

% Augmment System
Gz = [Gd,  zeros(6,2);
      -Cr, eye(2)];
Hz = [Hd;
      zeros(2,2)];

% Q and R tuning for Ki
Q8 = blkdiag(Q6, diag([20, 20]));  
R8 = eye(2)*30;

[Kz, ~, ~] = dlqr(Gz, Hz, Q8, R8);
Ki = Kz(:, 7:8); % Extract Ki

%% Simulate Nonlinear Closed Loop With Controller
clear functions
t_cl       = 0:Ts:80;                                          % Simulation time vector [0 to 80s in steps of Ts]
u_bar      = [0; m*g];                                         % equilibrium control input: F1=0N, F2=mg (hover thrust)
Controller = IntegralController(Kd, Ki, x_bar, u_bar, Cr, Ts); % Using Integral Action contorller

% Run Nonlinear Simulations
[t_sim0, x_sim0, u_sim0] = System_Simulator(@VTOL_Dynamics, x0, ...     % WITHOUT disturbance
                          t_cl, Controller, Ts, 0, @(t,x)x);

clear functions
[t_sim, x_sim, u_sim] = System_Simulator(@VTOL_Dynamics, x0, ...        % WITH disturbance
                          t_cl, Controller, Ts, student_ID, @(t,x)x);

% Check Gains Matrices 
fprintf('Kd:\n'); disp(Kd)
fprintf('Ki:\n'); disp(Ki)

% Force Checks (Complies To actuator constraint)
printForceAnalysis(u_sim0, u_sim, x0, x_bar, Kd, u_bar, 'Full-State NL+IC')

% Performance Metrics 
printPerformanceMetrics(t_sim0, x_sim0, t_sim, x_sim, x_bar, 'Full-State', 1, 3)

%% Plotting Nonlinear Close Loop Simulation (Shows Controller Works & Rejects Distrubances) 
figure;
sgtitle('NonLinear Closed Loop Simulation - With vs Without Disturbance');
subplot(3,2,1);
plot(t_sim0, x_sim0(:,1), 'b', t_sim, x_sim(:,1), 'g--');
ylim([0, 18]);
yline(x_bar(1), 'r--');
xlabel('Time (s)'); ylabel('X Position (m)');
title('Horizontal Position');
legend('No Disturbance', 'Disturbance', 'Target', 'Location', 'southeast');
subplot(3,2,2);
plot(t_sim0, x_sim0(:,2), 'b', t_sim, x_sim(:,2), 'g--');
xlabel('Time (s)'); ylabel('X Velocity (m/s)');
title('Horizontal Velocity');
legend('No Disturbance', 'Disturbance');
subplot(3,2,3);
plot(t_sim0, x_sim0(:,3), 'b', t_sim, x_sim(:,3), 'g--');
yline(x_bar(3), 'r--');
xlabel('Time (s)'); ylabel('Y Position (m)');
title('Altitude');
legend('No Disturbance', 'Disturbance', 'Target', 'Location', 'southeast');
subplot(3,2,4);
plot(t_sim0, x_sim0(:,4), 'b', t_sim, x_sim(:,4), 'g--');
ylim([-1, 8.5]);
xlabel('Time (s)'); ylabel('Y Velocity (m/s)');
title('Vertical Velocity');
legend('No Disturbance', 'Disturbance');
subplot(3,2,5);
plot(t_sim0, x_sim0(:,5)*180/pi, 'b', t_sim, x_sim(:,5)*180/pi, 'g--');
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Angle');
legend('No Disturbance', 'Disturbance', 'Location', 'southeast');
subplot(3,2,6);
plot(t_sim0, x_sim0(:,6)*180/pi, 'b', t_sim, x_sim(:,6)*180/pi, 'g--');
ylim([-28, 7]);
xlabel('Time (s)'); ylabel('Angular Rate (deg/s)');
title('Angular Rate');
legend('No Disturbance', 'Disturbance', 'Location', 'southeast');
saveas(gcf, 'NLControllerD&nD.png');

% Plotting Linear System Closed Loop Poles (shows Controller Stabilises System)
CL_sys = ss(Gd - Hd*Kd, Hd, C, D, Ts);   % closed loop system with Kd

figure;
sgtitle('Linearised System, closed loop Pole-Zero Map');
ax1 = subplot(1,2,1);
pzplot(ax1, CL_sys);
title('Full Unit Circle')
ax1.XLim = [-1.2, 1.2];
ax1.YLim = [-1.2, 1.2];
ax2 = subplot(1,2,2);
pzplot(ax2, CL_sys);
title('Zoomed In Pole Locations')
ax2.XLim = [0, 1.2];
ax2.YLim = [-0.3, 0.3];
saveas(gcf, 'CLsystemPoleMap.png')

% Print closed loop pole % stability check for report
printPoles(Gd, Hd, Kd,  'Closed Loop with Kd')
%% Observer Design 
% Observability Check
fprintf('\n') % Add space in command window
if rank(obsv(Gd,C))==size(Gd,1)
    disp('System is observable')
else
    disp('System is unobservable')
end
fprintf('\n') % Add space in command window

% Recompute observer poles 
ctrl_poles_cont = log(DesiredPoles) / Ts;   % Convert to continous time
obs_poles_cont  = 10 * ctrl_poles_cont;     % OBS pole 10x controller poles
obs_poles_disc  = exp(obs_poles_cont * Ts); % Discretise

% Scaling matrix
S = diag([m, m, m, m, J, J]);

% Output matrix for physical observer states/scaled measurements
C_phys = [m, 0, 0, 0, 0, 0;
         0, 0, m, 0, 0, 0;
         0, 0, 0, 0, J, 0];

% Controller gain scaled for physical state estimates
kd_phys = Kd * S;

% Cr converted to integrate physical position error correctly  
Cr_phys = diag([m, m]) * Cr;   % = [m,0,0,0,0,0; 0,0,m,0,0,0]

% Physical equilibrium
x_bar_phys = [x_ref; 0; y_ref; 0; 0; 0];  % [3;0;3;0;0;0]

% Compute L with C_phys
L = place(Gd_phys', C_phys', obs_poles_disc)';

%% Run simulations
% WITHOUT Disturbance
clear functions
Controller_obs = ObserverIntegralController(Gd_phys, Hd_phys, C_phys, ...
                  zeros(3,2), L, kd_phys, Ki, Cr_phys, x_bar_phys, u_bar, Ts);

[t_obs0, y_obs0, u_obs0] = System_Simulator(@VTOL_Dynamics, x0, t_cl, ...
                            Controller_obs, Ts, 0);

% WITH Disturbance
clear functions
Controller_obs = ObserverIntegralController(Gd_phys, Hd_phys, C_phys, ...
                  zeros(3,2), L, kd_phys, Ki, Cr_phys, x_bar_phys, u_bar, Ts);

[t_obs, y_obs, u_obs] = System_Simulator(@VTOL_Dynamics, x0, t_cl, ...
                          Controller_obs, Ts, student_ID);

% Linearised Closed-Loop Simulation with Kd only
delta_x0 = x0 - x_bar;
ut_cl    = zeros(length(t_cl), 2);
CL_ss    = ss(Gd - Hd*Kd, Hd, eye(6), zeros(6,2), Ts);
x_lin_cl = lsim(CL_ss, ut_cl, t_cl, delta_x0);

% Recover absolute states from deviation for plotting
x1_lin = x_lin_cl(:,1) + x_bar(1);   % X position
x2_lin = x_lin_cl(:,2);              % X velocity
x3_lin = x_lin_cl(:,3) + x_bar(3);   % Y position
x4_lin = x_lin_cl(:,4);              % Y velocity
x5_lin = x_lin_cl(:,5) * 180/pi;     % Angle (deg)
x6_lin = x_lin_cl(:,6) * 180/pi;     % Angular rate (deg/s)

% Observer velocity approximation (differentiate measured outputs)
% Observer only returns 3 measured outputs [x, y, theta]
% Velocities approximated by finite difference for plotting only

% WITHOUT disturbance model
x2_obs0 = [0; diff(y_obs0(:,1)) ./ diff(t_obs0)];           % X velocity
x4_obs0 = [0; diff(y_obs0(:,2)) ./ diff(t_obs0)];           % Y velocity
x6_obs0 = [0; diff(y_obs0(:,3)) ./ diff(t_obs0)] * 180/pi;  % Angular Rate

% WITH disturbance model
x2_obs  = [0; diff(y_obs(:,1))  ./ diff(t_obs)];            % X veloctiy
x4_obs  = [0; diff(y_obs(:,2))  ./ diff(t_obs)];            % Y velocity
x6_obs  = [0; diff(y_obs(:,3))  ./ diff(t_obs)]  * 180/pi;  % Angular Rate

%% Plotting model comparions WITHOUT Disturbance (Shows working OBS controller)
% Compares L+IC vs NL+IC vs NL+OBS+IC without disturbance

lw      = 1.5;             % Line Width
col_lin = [0.2 0.2 0.2];   % dark grey — Linearised + IC
col_nl  = [0   0.4 0.8];   % blue      — NL + IC
col_obs = [0.8 0.1 0.1];   % red       — NL + OBS + IC

% Plotting model comparisons
figure;
sgtitle('Model Comparison: L+IC vs NL+IC vs NL+OBS+IC (No Disturbance)');
subplot(3,2,1);
plot(t_cl,   x1_lin,             '-', 'Color', col_lin, 'LineWidth', lw); hold on
plot(t_sim0, x_sim0(:,1),        '-', 'Color', col_nl,  'LineWidth', lw);
plot(t_obs0, y_obs0(:,1),        '-', 'Color', col_obs, 'LineWidth', lw);
ylim([0, 20]);
yline(x_bar(1), 'k:', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('X Position (m)');
title('Horizontal Position');
legend('L+IC', 'NL+IC', 'NL+OBS+IC', 'Target', 'Location', 'southeast');
subplot(3,2,2);
plot(t_cl,   x2_lin,             '-', 'Color', col_lin, 'LineWidth', lw); hold on
plot(t_sim0, x_sim0(:,2),        '-', 'Color', col_nl,  'LineWidth', lw);
plot(t_obs0, x2_obs0,            '-', 'Color', col_obs, 'LineWidth', lw);
ylim([-2, 9]);
xlabel('Time (s)'); ylabel('X Velocity (m/s)');
title('Horizontal Velocity');
legend('L+IC', 'NL+IC', 'NL+OBS+IC', 'Location', 'northeast');
subplot(3,2,3);
plot(t_cl,   x3_lin,             '-', 'Color', col_lin, 'LineWidth', lw); hold on
plot(t_sim0, x_sim0(:,3),        '-', 'Color', col_nl,  'LineWidth', lw);
plot(t_obs0, y_obs0(:,2),        '-', 'Color', col_obs, 'LineWidth', lw);
yline(x_bar(3), 'k:', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Y Position (m)');
title('Altitude');
legend('L+IC', 'NL+IC', 'NL+OBS+IC', 'Target', 'Location', 'southeast');
subplot(3,2,4);
plot(t_cl,   x4_lin,             '-', 'Color', col_lin, 'LineWidth', lw); hold on
plot(t_sim0, x_sim0(:,4),        '-', 'Color', col_nl,  'LineWidth', lw);
plot(t_obs0, x4_obs0,            '-', 'Color', col_obs, 'LineWidth', lw);
ylim([0, 8]);
xlabel('Time (s)'); ylabel('Y Velocity (m/s)');
legend('L+IC', 'NL+IC', 'NL+OBS+IC', 'Location', 'northeast');
subplot(3,2,5);
plot(t_cl,   x5_lin,             '-', 'Color', col_lin, 'LineWidth', lw); hold on
plot(t_sim0, x_sim0(:,5)*180/pi, '-', 'Color', col_nl,  'LineWidth', lw);
plot(t_obs0, y_obs0(:,3)*180/pi, '-', 'Color', col_obs, 'LineWidth', lw);
yline(0, 'k:', 'LineWidth', 1.2);
title('Vertical Velocity');
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Roll Angle');
legend('L+IC', 'NL+IC', 'NL+OBS+IC', 'Location', 'southeast');
subplot(3,2,6);
plot(t_cl,   x6_lin,             '-', 'Color', col_lin, 'LineWidth', lw); hold on
plot(t_sim0, x_sim0(:,6)*180/pi, '-', 'Color', col_nl,  'LineWidth', lw);
plot(t_obs0, x6_obs0,            '-', 'Color', col_obs, 'LineWidth', lw);
ylim([-30, 15]);
yline(0, 'k:', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Angular Rate (deg/s)');
title('Angular Rate');
legend('L+IC', 'NL+IC', 'NL+OBS+IC', 'Location', 'southeast');
saveas(gcf, 'ModeComparisons.png')

% Plotting Disturbance Rejection — NL+OBS+IC Only (Shows the complete output feedback system rejecting disturbances)
figure;
sgtitle('NL+OBS+IC — Disturbance Rejection');
subplot(3,2,1);
plot(t_obs0, y_obs0(:,1),        'b',  'LineWidth', lw); hold on
plot(t_obs,  y_obs(:,1),         'g--','LineWidth', lw);
yline(x_bar(1), 'r--', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('X Position (m)');
title('Horizontal Position');
legend('No Disturbance', 'Disturbance', 'Target', 'Location', 'southeast');
subplot(3,2,2);
plot(t_obs0, x2_obs0,            'b',  'LineWidth', lw); hold on
plot(t_obs,  x2_obs,             'g--','LineWidth', lw);
ylim([-3 4]);
xlabel('Time (s)'); ylabel('X Velocity (m/s)');
title('Horizontal Velocity');
legend('No Disturbance', 'Disturbance');
subplot(3,2,3);
plot(t_obs0, y_obs0(:,2),        'b',  'LineWidth', lw); hold on
plot(t_obs,  y_obs(:,2),         'g--','LineWidth', lw);
yline(x_bar(3), 'r--', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Y Position (m)');
title('Altitude');
legend('No Disturbance', 'Disturbance', 'Target', 'Location', 'southeast');
subplot(3,2,4);
plot(t_obs0, x4_obs0,            'b',  'LineWidth', lw); hold on
plot(t_obs,  x4_obs,             'g--','LineWidth', lw);
ylim([-0.5 7.5]);
xlabel('Time (s)'); ylabel('Y Velocity (m/s)');
title('Vertical Velocity');
legend('No Disturbance', 'Disturbance');
subplot(3,2,5);
plot(t_obs0, y_obs0(:,3)*180/pi, 'b',  'LineWidth', lw); hold on
plot(t_obs,  y_obs(:,3)*180/pi,  'g--','LineWidth', lw);
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Roll Angle');
legend('No Disturbance', 'Disturbance', 'Location', 'southeast');
subplot(3,2,6);
plot(t_obs0, x6_obs0,            'b',  'LineWidth', lw); hold on
plot(t_obs,  x6_obs,             'g--','LineWidth', lw);
ylim([-22 6]);
yline(0, 'r--', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Angular Rate (deg/s)');
title('Angular Rate');
legend('No Disturbance', 'Disturbance', 'Location', 'southeast');
saveas(gcf, 'ObserverDisturbanceRejection.png')

%% Observer Performance Summary
printForceAnalysis(u_obs0, u_obs, x0, x_bar_phys, kd_phys, u_bar, 'Observer NL+OBS+IC')
printPerformanceMetrics(t_obs0, y_obs0, t_obs, y_obs, x_bar, 'Observer', 1, 2)

% Actuator Force Plots — NL+OBS+IC (Shows within motor constraints)
figure;
sgtitle('Acuator Forces — NL+OBS+IC');
subplot(3,1,1);
plot(t_obs0, u_obs0(:,1), 'b', 'LineWidth', lw); hold on
plot(t_obs,  u_obs(:,1),  'g--', 'LineWidth', lw);
yline(40,  'r--', 'LineWidth', 1.2);
yline(-40, 'r--', 'LineWidth', 1.2);
ylabel('Force (N)'); title('F1 — Horizontal Force');
legend('No Disturbance', 'Disturbance');
xlim([0 40]); ylim([-45 45])

subplot(3,1,2);
plot(t_obs0, u_obs0(:,2), 'b', 'LineWidth', lw); hold on
plot(t_obs,  u_obs(:,2),  'g--', 'LineWidth', lw);
yline(80, 'r--', 'LineWidth', 1.2);
yline(0,  'r--', 'LineWidth', 1.2);
ylabel('Force (N)'); title('F2 — Vertical Force');
legend('No Disturbance', 'Disturbance');
xlim([0 40]); ylim([-5 85]);

subplot(3,1,3);
plot(t_obs0, y_obs0(:,3)*180/pi, 'b', 'LineWidth', lw); hold on
plot(t_obs,  y_obs(:,3)*180/pi,  'g--', 'LineWidth', lw);
ylabel('Angle (deg)'); title('Roll Angle — Disturbance Rejected via Tilt');
legend('No Disturbance', 'Disturbance');
xlabel('Time (s)');
xlim([0 40]);
saveas(gcf, 'ActuatorForces.png');

% Observer vs Controller Pole-Zero Map
figure;
sgtitle('Controller vs Observer Poles');
ax1 = subplot(1,2,1);
hold(ax1, 'on');
% Plot unit circle
theta = linspace(0, 2*pi, 100);
plot(ax1, cos(theta), sin(theta), 'k--', 'LineWidth', 1);
% Controller poles
ctrl_p = eig(Gd - Hd*Kd);
plot(ax1, real(ctrl_p), imag(ctrl_p), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
% Observer poles
obs_p = eig(Gd_phys - L*C_phys);
plot(ax1, real(obs_p), imag(obs_p), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
xlabel('Real'); ylabel('Imaginary');
title('Full View');
legend('Unit Circle', 'Controller Poles', 'Observer Poles', 'Location', 'southwest');
axis equal; grid on;
xlim([-1.5 1.5]); ylim([-1.5 1.5]);

ax2 = subplot(1,2,2);
hold(ax2, 'on');
plot(ax2, cos(theta), sin(theta), 'k--', 'LineWidth', 1);
plot(ax2, real(ctrl_p), imag(ctrl_p), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
plot(ax2, real(obs_p),  imag(obs_p),  'ro', 'MarkerSize', 8,  'LineWidth', 2);
xlabel('Real'); ylabel('Imaginary');
title('Zoomed — Near Origin');
legend('Unit Circle', 'Controller Poles', 'Observer Poles', 'Location', 'southwest');
axis equal; grid on;
xlim([0.2 1]); ylim([-0.5 0.5]);
saveas(gcf, 'CtrlVObsPoles.png');

%% Animation — NL+OBS+IC with disturbance
clear functions

% Create observer controller
Controller_obs_anim = ObserverIntegralController(Gd_phys, Hd_phys, C_phys, ...
                       zeros(3,2), L, kd_phys, Ki, Cr_phys, x_bar_phys, u_bar, Ts);

% Run with @(t,x)x to get full 6 states back for animation
[t_anim, x_anim, ~] = System_Simulator(@VTOL_Dynamics, x0, t_cl, ...
                        @(t,x) Controller_obs_anim(t, Ce*x), ...
                        Ts, student_ID, @(t,x)x);

% Animate
Animate_VTOL(t_anim, x_anim, r)

%% Robustness Testing — Disturbances x Initial Conditions
test_IDs  = [54812365, 87654329];  % Different disturbances

% Difference Inital Postions
x0_tests  = {
    x0,                          '(0,0)';
    [m*1.5; 0; m*1.5; 0; 0; 0], '(1.5,1.5)';
    [m*(-1); 0; m*1; 0; 0; 0],  '(-1,1)';
};

% Line styles per starting position
line_styles = {'-', '--', ':'};

% Colours per disturbance ID
colours = [0.5 0.0 0.8;   % purple
           0.1 0.7 0.1];  % green

% Storage
results_t = cell(length(test_IDs), size(x0_tests,1));
results_y = cell(length(test_IDs), size(x0_tests,1));

% Run all combinations of disturbances and starting postions
for i = 1:length(test_IDs)
    for j = 1:size(x0_tests,1)
        clear functions
        Controller_rob = ObserverIntegralController(Gd_phys, Hd_phys, C_phys, ...
                          zeros(3,2), L, kd_phys, Ki, Cr_phys, x_bar_phys, u_bar, Ts);

        [t_rob, y_rob, ~] = System_Simulator(@VTOL_Dynamics, x0_tests{j,1}, t_cl, ...
                             Controller_rob, Ts, test_IDs(i));

        results_t{i,j} = t_rob;
        results_y{i,j} = y_rob;
    end
end

% Plotting 
figure;
sgtitle('Robustness Testing — Multiple Disturbances and Initial Conditions');

subplot(2,1,1); hold on;
for i = 1:length(test_IDs)
    for j = 1:size(x0_tests,1)
        label = sprintf('ID:%d | x0=%s', test_IDs(i), x0_tests{j,2});
        plot(results_t{i,j}, results_y{i,j}(:,1), ...
             line_styles{j}, 'Color', colours(i,:), ...
             'LineWidth', 1.5, 'DisplayName', label);
    end
end
yline(x_bar(1), 'r--', 'LineWidth', 1.2, 'DisplayName', 'Target');
xlabel('Time (s)'); ylabel('X Position (m)');
title('Horizontal Position');
legend('Location', 'southeast', 'FontSize', 7);
ylim([-6 22]);

subplot(2,1,2); hold on;
for i = 1:length(test_IDs)
    for j = 1:size(x0_tests,1)
        plot(results_t{i,j}, results_y{i,j}(:,2), ...
             line_styles{j}, 'Color', colours(i,:), ...
             'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
end
yline(x_bar(3), 'r--', 'LineWidth', 1.2, 'DisplayName', 'Target');
xlabel('Time (s)'); ylabel('Y Position (m)');
title('Altitude');
legend('Location', 'southeast');
saveas(gcf, 'RobustnessTesting.png');

%% Extensions Kalman Filter Design and Comparison
L_scaled = place(Gd', C', obs_poles_disc)'; % L designed on physical system — need equivalent on scaled system for lsim

% Noise covariances Q & R tuning
Q_kf = diag([1, 1, 1, 1, 0.1, 0.1]);
R_kf = diag([0.05, 0.05, 0.005]);

% Equilibrium output
y_bar = C * x_bar;

% Use CLOSED LOOP system matrix — stable around equilibrium
Gd_cl = Gd - Hd*Kd;   % closed loop state matrix

% Small perturbation around equilibrium to show noise filtering
x_true = delta_x0;     % start from origin as before

% Initial error covariance for Kalman filter
P = eye(6);

% Initialise both observers at same point as true state for fair comparison
x_kf  = delta_x0;   % Kalman filter initial estimate
x_lue = delta_x0;   % Luenberger initial estimate

% Storage arrays 
N           = length(t_cl);
x_kf_log    = zeros(N,6);   % Kalman filter estimates
x_lue_log   = zeros(N,6);   % Luenberger estimates
x_real_log  = zeros(N,6);   % True states
y_noisy_log = zeros(N,3);   % Noisy measurements

% simulation loop — propagates true state and both observers forward
for k = 1:N
    % Real state — absolute coordinates
    x_real_log(k,:) = (x_true + x_bar)';

    % Noisy measurement
    noise   = sqrt(diag(R_kf)) .* randn(3,1);   % generate Gaussian noise
    y_noisy = C * x_true + noise;               % add it to the clean measurement
    y_noisy_log(k,:) = (y_noisy + y_bar)';      % log in absolute coordinates

    % Kalman Filter Update
    x_kf_pred = Gd_cl * x_kf;
    P_pred     = Gd_cl * P * Gd_cl' + Q_kf;

    % Update step — correct prediction using noisy measurement
    S_inn    = C * P_pred * C' + R_kf;           % Innovation covariance — total uncertainty
    K_kalman = P_pred * C' / S_inn;              % Optimal Kalman gain — balances model vs measurement trust
    innov    = y_noisy - C * x_kf_pred;          % Innovation — difference between measurement and prediction
    x_kf     = x_kf_pred + K_kalman * innov;     % Corrected state estimate
    P        = (eye(6) - K_kalman * C) * P_pred; % Updated covariance — shrinks after measurement

    x_kf_log(k,:) = (x_kf + x_bar)';

    % Luenberger Update
    delta_y  = y_noisy - C * x_lue;
    x_lue    = (Gd_cl - L_scaled*C) * x_lue + L_scaled * delta_y; % Observer update equation
    x_lue_log(k,:) = (x_lue + x_bar)';

    % Propagate true state with closed loop dynamics
    x_true = Gd_cl * x_true;
end

% Plotting (Shows why we need Kf filter)
% Figure 1 — Full simulation overview
figure;
sgtitle('Kalman Filter vs Luenberger — Full Simulation');
subplot(3,1,1);
plot(t_cl, x_real_log(:,1),  'm',   'LineWidth', 2.0); hold on
plot(t_cl, x_lue_log(:,1),   'r--', 'LineWidth', 1.0);
plot(t_cl, x_kf_log(:,1),    'b--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('X Position (m)');
title('X Position'); ylim([0 20]);
legend('Real', 'Luenberger', 'Kalman Filter', 'Location', 'southeast');

subplot(3,1,2);
plot(t_cl, x_real_log(:,3),  'm',   'LineWidth', 2.0); hold on
plot(t_cl, x_lue_log(:,3),   'r--', 'LineWidth', 1.0);
plot(t_cl, x_kf_log(:,3),    'b--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Y Position (m)');
title('Y Position');
legend('Real', 'Luenberger', 'Kalman Filter', 'Location', 'southeast');

subplot(3,1,3);
plot(t_cl, x_real_log(:,5)*180/pi,  'm',   'LineWidth', 2.0); hold on
plot(t_cl, x_lue_log(:,5)*180/pi,   'r--', 'LineWidth', 1.0);
plot(t_cl, x_kf_log(:,5)*180/pi,    'b--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Roll Angle');
legend('Real', 'Luenberger', 'Kalman Filter', 'Location', 'northeast');
saveas(gcf, 'KalmanFilter_Full.png');

% Figure 2 — Zoomed in at steady state to show noise difference clearly
t_zoom_start = 30;   % zoom into settled region
t_zoom_end   = 40;   % 10 second window

figure;
sgtitle('Kalman Filter vs Luenberger — Steady State Zoom (t=30-40s)');
subplot(3,1,1);
plot(t_cl, x_real_log(:,1),  'm',   'LineWidth', 2.0); hold on
plot(t_cl, x_lue_log(:,1),   'r--', 'LineWidth', 1.0);
plot(t_cl, x_kf_log(:,1),    'b--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('X Position (m)');
title('X Position');
xlim([t_zoom_start t_zoom_end]);
legend('Real', 'Luenberger', 'Kalman Filter', 'Location', 'best');

subplot(3,1,2);
plot(t_cl, x_real_log(:,3),  'm',   'LineWidth', 2.0); hold on
plot(t_cl, x_lue_log(:,3),   'r--', 'LineWidth', 1.0);
plot(t_cl, x_kf_log(:,3),    'b--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Y Position (m)');
title('Y Position');
xlim([t_zoom_start t_zoom_end]);
legend('Real', 'Luenberger', 'Kalman Filter', 'Location', 'best');

subplot(3,1,3);
plot(t_cl, x_real_log(:,5)*180/pi,  'm',   'LineWidth', 2.0); hold on
plot(t_cl, x_lue_log(:,5)*180/pi,   'r--', 'LineWidth', 1.0);
plot(t_cl, x_kf_log(:,5)*180/pi,    'b--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Roll Angle');
xlim([t_zoom_start t_zoom_end]);
legend('Real', 'Luenberger', 'Kalman Filter', 'Location', 'best');
saveas(gcf, 'KalmanFilter_Zoom.png');

% Note:it's running the linearised closed loop model in a for loop with artificially injected noise. 
% The black box isn't involved at all here, showing what would happen to
% obs if there was noise 