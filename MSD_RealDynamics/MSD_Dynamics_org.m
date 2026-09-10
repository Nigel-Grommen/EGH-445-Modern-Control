function dxdt = MSD_Dynamics(t, x, F_input, student_ID)
%nonlinear_msd_dynamics Defines the state-space dynamics of a nonlinear mass-spring-damper system.
%
%   Equation of Motion:
%       m*q_ddot + b*q_dot*abs(q_dot) + k*q = F
%
%   State Vector:
%       x = [q; q_dot] where q is position, q_dot is velocity.
%
%   State Derivative Vector (Output):
%       dxdt = [q_dot; q_ddot]
%
%   Inputs:
%       t       : Current time (scalar). Often unused if F is constant or only state-dependent.
%       x       : Current state vector [q; q_dot] (2x1 column vector).
%       F_input : External force. Can be a scalar value or a function handle
%                 expecting inputs (t, x), e.g., F_input = @(time, state) sin(time).
%
%   Output:
%       dxdt    : State derivative vector [q_dot; q_ddot] (2x1 column vector).
%
%   Author: Guilherme Froes Silva
%   Date: 2025-03-26

coder.allowpcode('plain')

% --- Parameters ---
m = 1;
b = 1;
k = 1;
dist = 1;

% --- Input Validation (Basic) ---
if ~isvector(x) || numel(x) ~= 2
    error('State vector x must be a 2-element vector [q; q_dot].');
end
% Ensure x is a column vector for consistency
x = x(:); 

if nargin < 4
    student_ID = 0;
end
if ~isscalar(student_ID) || floor(student_ID) ~= student_ID
    error('student_ID must be an integer.');
end

% --- Evaluate External Force ---
% This allows F to be passed as a constant or a function of time/state
if isa(F_input, 'function_handle')
    F = F_input(t, x); % Evaluate force if it's a function handle
else
    F = F_input;       % Use force directly if it's a constant/value
end

% --- Extract States ---
q = x(1);      % Position
q_dot = x(2);  % Velocity

% Personalisation
d_u = 0;
m_actual = m;
k_actual = k;
b_actual = b;

if student_ID ~=0
    % Use student ID for reproducible randomness or direct variation
    % rng(student_ID); % Seed the random number generator
    
    % Parameter Uncertainty (example only)
    last_digit = mod(student_ID, 10);
    uncertainty_factor = 1 + (last_digit - 4.5) / 45; % Creates a factor roughly between 0.9 and 1.1
    m_actual = m * uncertainty_factor;
    b_actual = b * (1 / uncertainty_factor); % Example inverse variation
    % k_actual = k * (1 + (rand - 0.5) * 0.2); % +/- 10% random variation seeded by ID
    
    % Input Disturbance (Example: Constant offset + small sine wave) ---
    % Use digits from student ID to define disturbance
    disturbance_offset = mod(student_ID, 5) * 0.1 - 0.2; % Offset between -0.2 and +0.2 N
    disturbance_amp = mod(student_ID, 5) * 0.05; % Amplitude up to 0.2 N
    disturbance_freq = 1 + mod(student_ID, 5); % Frequency between 1 and 5 rad/s
    d_u = disturbance_offset + disturbance_amp * sin(disturbance_freq * t);
end


% --- Calculate State Derivatives ---
% dx1/dt = q_dot
dqdt = q_dot;

% dx2/dt = q_ddot = (1/m) * (F - b*q_dot*abs(q_dot) - k*q)
dq_dot_dt = (1/m_actual) * (F+d_u - b_actual * q_dot * abs(q_dot) - k_actual * q) ;%+ dist;

% --- Assemble Output Vector ---
% The output must be a column vector for ODE solvers
dxdt = [dqdt; dq_dot_dt];

end