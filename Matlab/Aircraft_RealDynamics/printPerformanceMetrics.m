function printPerformanceMetrics(t_nd, x_nd, t_d, x_d, x_bar, label, col_x, col_y)
% printPerformanceMetrics  Computes and prints time-domain performance metrics
%
%   This function compares system performance for:
%   - No disturbance case
%   - Disturbance case
%
%   It evaluates overshoot, undershoot, and settling time for:
%   - X position
%   - Y position
%
% INPUTS:
%   t_nd   : Time vector (no disturbance)
%   x_nd   : State/output matrix (no disturbance)
%   t_d    : Time vector (disturbance case)
%   x_d    : State/output matrix (disturbance case)
%   x_bar  : Equilibrium state vector [x; x_dot; y; y_dot; theta; theta_dot]
%   label  : String label for display (e.g. 'Observer Controller')
%   col_x  : Column index for x position (default = 1)
%   col_y  : Column index for y position (default = 3)
%
% OUTPUTS:
%   (none) - results printed to command window
%
% NOTES:
%   - Settling time uses a 2% tolerance band
%   - Handles overshoot and undershoot separately
%   - Designed for both full-state and observer-based outputs

% Use default columns if none given
if nargin < 7
    col_x = 1;
    col_y = 3;
end

fprintf('\n=== Performance Metrics — %s ===\n', label)

% Target positions
target_x = x_bar(1);   % desired x position
target_y = x_bar(3);   % desired y position

% 2% settling bands
band_x = 0.02 * abs(target_x);   % x tolerance
band_y = 0.02 * abs(target_y);   % y tolerance


% Store no-disturbance and disturbance data together
data = {t_nd, x_nd, 'No Disturbance';
        t_d,  x_d,  'With Disturbance'};

% Loop through both cases
for i = 1:2

    t_use = data{i,1};  % Extract time vector
    x_use = data{i,2};  % Extract state/output matrix

    fprintf('\n--- %s ---\n', data{i,3})

    % X position metrics
    x_pos = x_use(:, col_x);

    % Peak value
    peak_x = max(x_pos);

    % Settling time
    idx = find(abs(x_pos - target_x) > band_x, 1, 'last');  % Find last point outside settling band

    % Check if system settled
    if isempty(idx) || idx == length(t_use)
        ST_x = NaN;
    else
        ST_x = t_use(idx);
    end

    fprintf('X Position:\n')
    fprintf('  Peak Value: %.2f m\n', peak_x)

    % Overshoot / undershoot logic
    if peak_x > target_x

        OS_x = ((peak_x - target_x) / abs(target_x)) * 100;

        fprintf('  Overshoot: %.1f%%\n', OS_x)

    else

        min_x = min(x_pos);

        if min_x < target_x
            US_x = ((target_x - min_x) / abs(target_x)) * 100; % Calculate undershoot percentage

            fprintf('  No overshoot\n')
            fprintf('  Undershoot: %.1f%%\n', US_x)
        else
            fprintf('  Monotonic response (no over/undershoot)\n')
        end
    end

    % Settling time print
    if isnan(ST_x)
        fprintf('  Settling: did not settle within simulation\n')
    else
        fprintf('  Settling Time: %.2f s (2%% band = %.2f m)\n', ...
                ST_x, band_x)
    end

    % Y position metrics
    y_pos = x_use(:, col_y);

    % Peak value
    peak_y = max(y_pos);

    % Settling time
    idy = find(abs(y_pos - target_y) > band_y, 1, 'last');

    if isempty(idy) || idy == length(t_use)
        ST_y = NaN;
    else
        ST_y = t_use(idy);
    end

    fprintf('Y Position:\n')
    fprintf('  Peak Value: %.2f m\n', peak_y)

    % Overshoot / undershoot logic
    if peak_y > target_y

        OS_y = ((peak_y - target_y) / abs(target_y)) * 100;

        fprintf('  Overshoot: %.1f%%\n', OS_y)

    else

        min_y = min(y_pos);

        if min_y < target_y
            US_y = ((target_y - min_y) / abs(target_y)) * 100;

            fprintf('  No overshoot\n')
            fprintf('  Undershoot: %.1f%%\n', US_y)
        else
            fprintf('  Monotonic response (no over/undershoot)\n')
        end
    end

    % Settling time print
    if isnan(ST_y)
        fprintf('  Settling: did not settle within simulation\n')
    else
        fprintf('  Settling Time: %.2f s (2%% band = %.2f m)\n', ...
                ST_y, band_y)
    end

end
end