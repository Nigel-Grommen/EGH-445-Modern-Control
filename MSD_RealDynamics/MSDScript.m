clear all; close all; 

%% You MUST define those variables such that the protected model works
student_ID = 11908203;

%% Parameters
m = 1; % Mass [kg]
b = 1; % Damping coefficient [N s^2/m^2]
k = 1; % Spring coefficient [N/m]

%% Equilibrium Points
u_bar = 5;

syms x1 x2 u real
x_dot_1 = x2;
x_dot_2 = -b*x2*abs(x2) - k*x1 + u;
eq1e = subs(x_dot_1, u, u_bar);
eq2e = subs(x_dot_2, u, u_bar);
x_bar = solve([eq1e, eq2e], [x1, x2]);
x_bar = double([x_bar.x1 x_bar.x2]);
x_bar = x_bar(:); % ensure column

%% Linearisation
A = jacobian([x_dot_1, x_dot_2], [x1, x2]);
Ae = double(subs(A, [x1; x2], x_bar));
Be = double(jacobian([x_dot_1, x_dot_2], u));
Ce = [1 0];
De = 0;

MSD_Linearised = ss(Ae, Be, Ce, De);

%% Open-loop simulation - Initial Conditions
Ts = 0.001; % Sampling Time [s]
t = 0:Ts:5;
ut = zeros(size(t));
x0 = [1; 0] + x_bar;
delta_x0 = x0 - x_bar;

% Linearised System
ylin = lsim(MSD_Linearised, ut, t, delta_x0) + x_bar(1); % + x_bar.x1 to recover the absolute value of x1 (instead of delta x1)
f1 = figure(1);clf;
plot(t, ylin, 'r')
legend('Linear')
xlabel('Time (seconds)')
ylabel('Position (metres)')

% Nonlinear System
F_input = @(t, x) u_bar;
[t, x] = ode45(@(t, x) MSD_Dynamics(t, x, F_input, student_ID), t, x0);
y = x(:,1);
f2 = figure(2);clf;
plot(t, y, 'b', t, ylin, 'c') % plotting linearised system for comparison
xlabel('Time (seconds)')
ylabel('Position (metres)')
title('Nonlinear vs linearised Simulation Results')
legend('Nonlinear', 'Linear')

%% Control Design
% Discretisation
sysD = c2d(MSD_Linearised, Ts, 'zoh');
G = sysD.A; H = sysD.B; C = sysD.C; D = sysD.D;

% Controllability Check
Ctrb = ctrb(G,H);
rank(Ctrb);
desiredPoles_cont = [-2 -3];
desiredPoles = exp(desiredPoles_cont*Ts);
Kd = place(G, H, desiredPoles);

% Linearised Closed-loop System
MSD_Linearised_ClosedLoop = ss(G-H*Kd, 0*H, eye(2), D, Ts); % 0*Be to retain size of Be
xlinCL = lsim(MSD_Linearised_ClosedLoop, ut, t, delta_x0);
ylinCL = C*xlinCL' + x_bar(1);
ulinCL = -Kd*xlinCL' + u_bar; 

f3 = figure(3);clf;
plot(t, ylinCL, 'c'); hold on
stairs(t, ulinCL, 'm') % plotting linearised system for comparison
hold off
xlabel('Time (seconds)')
ylabel('Position (metres)')
title('Linearised Model Simulation (state-feedback control)')
legend(["Position", "Control"])

% Nonlinear Closed-loop System
Controller = @(t, x) -Kd*(x-x_bar) + u_bar;
[t_cl, x_cl, u_cl] = System_Simulator(@MSD_Dynamics, x0, t, Controller, Ts, student_ID, @(t,x)x); 
% By adding @(t,x)x as the last parameter, you allow the controller to 
% access x directly, the Simulator also outputs x instead of y

y_cl = x_cl(:,1);
f4 = figure(4);clf;
plot(t, y_cl, 'b'); hold on
plot(t, ylinCL, 'c--');
stairs(t, u_cl, 'r'); % plotting linearised system for comparison
stairs(t, ulinCL, 'm--'); % plotting linearised system for comparison
hold off
xlabel('Time (seconds)')
ylabel('Position (metres)')
title('Nonlinear Simulation (state-feedback control)')
legend(["NL Position", "L Position", "NL Control", "L Control"])

%% Output-feedback Control
[t_cl, y_cl, u_cl] = System_Simulator(@MSD_Dynamics, x0, t, Controller, Ts, student_ID);
% Now with Ts as the last parameter, the Simulator only exposes access to the 
% output y. If we pass the full-state feedback controller handle, it becomes 
% u = -Ky, % which doesn't work.

f5 = figure(5);clf;
subplot(211)
plot(t_cl, y_cl, 'b', t, ylinCL, 'c') % plotting linearised system for comparison
xlabel('Time (seconds)')
ylabel('Position (metres)')
title('Nonlinear vs linearised Simulation Results')
legend('Nonlinear', 'Linear')
subplot(212)
stairs(t_cl, u_cl, 'r'); hold on
stairs(t, ulinCL, 'm')
legend(["NL Control", "L Control"])
hold off

%% Observer Design
% Observability Check 
Obsv = obsv(G,C);
rank(Obsv);
Ld = place(G', C', exp(10*desiredPoles_cont*Ts))';

%% Combine Controller + Observer
% clear functions
x_hat0 = x0 - x_bar;
delta_x_hat0 = x0 - x_bar;
Controller = createObserverController(G, H, C, D, Ld, Kd, x_hat0, x_bar, u_bar); % Most of your work will be on modifying this function! 
[t_sim, y_sim, u_sim] = System_Simulator(@MSD_Dynamics, x0, t, Controller, Ts, 1094132);

% Simulate Linearised Model with Observer
LinControllerObserverDynMatrix = [G-H*Kd H*Kd; zeros(2,2) G - Ld*C];
MSD_Linearised_ClosedLoop_Obsv = ss(LinControllerObserverDynMatrix, zeros(4,1), eye(4), D, Ts); % 0*Be to retain size of Be
xlinCLobsv = lsim(MSD_Linearised_ClosedLoop_Obsv, ut, t, [delta_x0; delta_x0-delta_x_hat0]);
ylinCLobsv = C*xlinCLobsv(:,1:2)' + x_bar(1);
ulinCLobsv = -Kd*xlinCLobsv(:,1:2)' + u_bar; 

f6 = figure(6);clf;
subplot(211)
plot(t_sim, y_sim, 'b'); hold on
plot(t_sim, ylinCLobsv, 'c');
legend(["NL Position", "L Position"])

subplot(212)
stairs(t_sim, u_sim, 'r'); hold on
stairs(t_sim, ulinCLobsv, 'm')
legend({"NL Control", "L Control"}) 
hold off

%% Using Simulink 
y_bar = x_bar(1);
out = sim("MSD_Model_Real.slx", 'stoptime', '5');

f7 = figure(7); clf;
subplot(211)
plot(out.y, 'b'); hold on
legend("NL Position")

subplot(212)
plot(out.u, 'r'); hold on
axis([0 5 0 7])
legend("NL Control") 
hold off

%% Using Simulink (Output feedback)

y_bar = x_bar(1);
out = sim("MSD_Model_Real_Protected.slx", 'stoptime', '5');
f8 = figure(8); clf;
subplot(211)
plot(out.y, 'b'); hold on
legend("NL Position")

subplot(212)
plot(out.u, 'r'); hold on
axis([0 5 0 7])
legend("NL Control") 
hold off

