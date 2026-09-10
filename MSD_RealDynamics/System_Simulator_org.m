function [T, Y, U_zoh] = System_Simulator(system_dynamics_handle, ...
    initial_state, t_span, control_law_handle, sample_time, student_ID, output_function_handle)
%System_Simulator_v2 Simulates a continuous-time system under discrete, ZOH control.
%
%   [T, Y, U_zoh] = System_Simulator_v2(system_dynamics_handle, ...
%       initial_state, t_span, control_law_handle, sample_time, output_function_handle)
%
%   Inputs:
%       system_dynamics_handle: Function handle for the continuous system dynamics.
%                               Expected signature: dxdt = f(t, x, u)
%                               where u is the control input.
%       initial_state:          Column vector of initial state values x(0).
%       t_span:                 Time interval for simulation, e.g., [t_start, t_end].
%       control_law_handle:     Function handle for the discrete control law.
%                               Expected signature: u = g(t, y)
%                               where y is the system output at sample time t.
%       sample_time:            Sampling period (Ts) for the discrete controller.
%       output_function_handle: (Optional) Function handle to compute system output y from state x.
%                               Expected signature: y = h(t, x)
%                               Defaults to y = x if not provided or empty.
%
%   Outputs:
%       T:      Column vector of time points from the ODE solver.
%       Y:      Matrix of system outputs at each time point in T (each row is y(t)').
%       U_zoh:  Matrix of the piecewise constant ZOH control input applied
%               at each time point in T (each row is u(t)').
%
%   Author: Guilherme Froes Silva (QUT)
%   Date: 2025-03-27

    % --- Input Validation & Defaults ---
    if nargin < 5
        error('Not enough input arguments.');
    end
    if ~isa(system_dynamics_handle, 'function_handle')
        error('system_dynamics_handle must be a function handle.');
    end
    switch class(control_law_handle)
        case 'function_handle'
        case 'double'
            control_law_handle = @(t,x) control_law_handle;
        otherwise
            error('control_law_handle must be a function handle or a constant.');
    end
       
    if ~isscalar(sample_time) || sample_time <= 0
        error('sample_time (Ts) must be a positive scalar.');
    end
    if ~isscalar(student_ID) || floor(student_ID) ~= student_ID
        error('student_ID must be an integer.');
    end
    if nargin < 7 || isempty(output_function_handle)
        % Default output function: output y is the full state x
        output_function_handle = @(t, x) x;
        % Compare if system_dynamics_handle is a specific function (e.g., MSD_Dynamics)
        % and set output function accordingly
        if strcmp(func2str(system_dynamics_handle), 'MSD_Dynamics')
            % Example of how sensor noise might be added. Potentially the
            % random noise will be more directly dependent on the
            % student_ID
            rng(student_ID)
            output_function_handle = @(t, x) x(1) + (rand-.5)/10; % Only position output
        end
    elseif ~isa(output_function_handle, 'function_handle')
         error('output_function_handle must be a function handle.');
    end

    % Ensure initial state is a column vector
    initial_state = initial_state(:);
    t_start = t_span(1);
    t_end = t_span(end);

    % --- Data Storage for ZOH signal history (filled by nested function) ---
    control_update_times = [];
    control_update_values = []; % Store control inputs as rows

    % --- Setup ODE Solver ---
    % Define the nested function handle passed to the ODE solver.
    % It encapsulates the ZOH logic.
    odefun = @(t, x) system_dynamics_zoh(t, x);

    % Include sample times in the simulation time vector to encourage
    % the ODE solver to evaluate the solution near these points.
    t_samples = (t_start:sample_time:t_end);
    t_span_augmented = unique([t_start, t_samples, t_end]); % Ensure start/end included
    
    % Sets the maximum step size to a fraction of the sample_time
    max_step = sample_time/2; % Or Ts/2, Ts/4 etc. depending on needs
    options = odeset('MaxStep', max_step);
    
    % --- Call ODE Solver ---
    % Using ode45, a versatile solver. Choose others (ode23, ode15s) if needed.
    [T, X_sol] = ode45(odefun, t_span_augmented, initial_state, options);

    % --- Reconstruct Output Y ---
    % Determine output size based on the first call
    y_dim_check = output_function_handle(T(1), X_sol(1,:)');
    num_outputs = numel(y_dim_check);
    Y = zeros(length(T), num_outputs); % Preallocate Y
    for i = 1:length(T)
        % Apply output function, ensure result is a row vector for storage
        y_row = output_function_handle(T(i), X_sol(i,:)')';
        Y(i,:) = y_row;
    end

    % --- Reconstruct ZOH Control Signal U_zoh ---
    if isempty(control_update_times)
        % If simulation was shorter than one Ts, or no updates occurred
        warning('No control updates recorded during simulation span.');
        % Calculate initial control based on initial conditions
        y_initial = output_function_handle(t_start, initial_state);
        u_initial = control_law_handle(t_start, y_initial);
        num_inputs = numel(u_initial);
        U_zoh = repmat(u_initial', length(T), 1); % Repeat initial control
    else
        num_inputs = size(control_update_values, 2);
        U_zoh = zeros(length(T), num_inputs); % Preallocate U
        for i = 1:length(T)
            % Find the index of the last control update time <= current time T(i)
            update_index = find(control_update_times <= T(i) + 1e-10, 1, 'last'); % Add tolerance
            if isempty(update_index)
                 % Should not happen if control_update_times is not empty and T(1) >= t_start
                 % Fallback to initial control calculation if needed
                 y_initial = output_function_handle(t_start, initial_state);
                 u_initial = control_law_handle(t_start, y_initial);
                 U_zoh(i,:) = u_initial';
            else
                U_zoh(i,:) = control_update_values(update_index, :);
            end
        end
    end

    % --- Nested Function Implementing ZOH Dynamics ---
    function dxdt = system_dynamics_zoh(t_current, x_current)
        % Manages control updates and calls the continuous system dynamics.

        % Use persistent variables *within this nested function* for ZOH state
        persistent p_last_calc_time p_last_u p_initialized;

        % Initialize on the very first call by ode45 or if time resets
        if isempty(p_initialized) || t_current == t_start
            y_now = output_function_handle(t_start, x_current); % Use current state at t_start
            p_last_u = control_law_handle(t_start, y_now); % Calculate initial control u(0)
            p_last_calc_time = t_start; % Time control u(0) was calculated
            p_initialized = true;

            % Clear and store initial control history (for external reconstruction)
            control_update_times = [p_last_calc_time];
            control_update_values = [p_last_u']; % Ensure stored as a row
            disp(['ZOH Init: t=', num2str(t_current), ', u=', num2str(p_last_u')]) % Debug
        end

        % Check if a new control signal needs to be calculated for the *next* interval
        % Calculate if current time is >= the start time of the next sample interval
        next_calc_time = p_last_calc_time + sample_time;
        if t_current >= next_calc_time - 1e-10 % Use small tolerance for floating point compare
            % Calculate control based on state at this time (or closest available sample time state if preferred)
            y_now = output_function_handle(t_current, x_current);
            u_new = control_law_handle(t_current, y_now);

            % Update the control value to be held
            p_last_u = u_new;
            % Update the time associated with this calculation (start of interval)
            p_last_calc_time = next_calc_time; % Or floor(t_current/sample_time)*sample_time? Use next_calc_time for consistency.

            % Store history for reconstruction later
            control_update_times = [control_update_times; p_last_calc_time]; % Store sample time
            control_update_values = [control_update_values; p_last_u']; % Store as row
            % disp(['ZOH Update: t_current=', num2str(t_current), ', calc_time=', num2str(p_last_calc_time), ', u=', num2str(p_last_u)]) % Debug
        end

        % Always apply the most recently calculated (and held) control input
        current_u = p_last_u;

        % --- Call the actual continuous system dynamics ---
        % Assumes system_dynamics_handle has signature: dxdt = f(t, x, u)
        dxdt = system_dynamics_handle(t_current, x_current, current_u, student_ID);

    end % End of nested function system_dynamics_zoh

    % Optional: Clear persistent variables of nested function if simulator might be
    % called multiple times where independent ZOH state is needed per call.
    % clear functions; % This clears *all* persistent variables, use with caution.
    % Alternatively, modify system_dynamics_zoh to accept and return ZOH state.

end % End of System_Simulator_v2