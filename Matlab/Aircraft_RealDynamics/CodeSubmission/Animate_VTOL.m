function Animate_VTOL(t, x_state, r)
    % ANIMATE_VTOL Animates the planar VTOL aircraft with a tracking camera.
    %
    % Inputs:
    %   t       : Time vector from the ODE solver.
    %   x_state : State matrix (N x 6) from the ODE solver.
    %   r       : Thrust offset/arm length (used for geometric scaling).

    % Extract position and orientation states
    pos_x = x_state(:, 1);
    pos_y = x_state(:, 3);
    theta = x_state(:, 5);

    % Set up the figure window
    fig = figure('Name', 'VTOL Animation', 'Color', 'w');
    ax = axes('Parent', fig);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    
    xlabel(ax, 'X Position (m)');
    ylabel(ax, 'Y Position (m)');
    
    % Define the tracking window size (metres from the centre of mass)
    win = 3; 

    % Initialise graphic objects once (improves rendering performance)
    h_trail = plot(ax, pos_x(1), pos_y(1), 'b--', 'LineWidth', 1.5);
    h_body  = plot(ax, [0 0], [0 0], 'k', 'LineWidth', 4);
    h_mast  = plot(ax, [0 0], [0 0], 'k', 'LineWidth', 4);
    h_cg    = plot(ax, 0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

    % Calculate frame skipping to maintain ~30 FPS
    dt = mean(diff(t));
    target_fps = 30;
    step = max(1, round(1 / (target_fps * dt)));

    for i = 1:step:length(t)
        % Exit gracefully if the user closes the window early
        if ~isgraphics(fig)
            break; 
        end
        
        % Current state variables
        cx = pos_x(i);
        cy = pos_y(i);
        th = theta(i);

        % Calculate VTOL geometry based on orientation
        left_x = cx - r * cos(th);
        left_y = cy - r * sin(th);
        right_x = cx + r * cos(th);
        right_y = cy + r * sin(th);
        
        mast_x = cx - r * sin(th);
        mast_y = cy + r * cos(th);

        % Update the graphic object data properties
        set(h_trail, 'XData', pos_x(1:i), 'YData', pos_y(1:i));
        set(h_body, 'XData', [left_x, right_x], 'YData', [left_y, right_y]);
        set(h_mast, 'XData', [cx, mast_x], 'YData', [cy, mast_y]);
        set(h_cg, 'XData', cx, 'YData', cy);
        
        % Update the tracking camera limits
        axis(ax, [cx - win, cx + win, cy - win, cy + win]);
        title(ax, sprintf('VTOL Nonlinear Dynamics \n Time: %.2f s', t(i)));

        drawnow;
    end
end