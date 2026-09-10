function controller_handle = ObserverIntegralController(Gd, Hd, Cd, Dd, L, Kd, Ki, Cr, x_bar, u_bar, Ts)

    persistent p_x_hat p_z_int p_last_u p_initialized

    x_bar = x_bar(:);
    u_bar = u_bar(:);
    y_bar = Cd * x_bar;   % = [12; 12; 0]

    if isempty(p_initialized)
        p_x_hat       = zeros(6,1);
        p_z_int       = zeros(2,1);
        p_last_u      = u_bar;
        p_initialized = true;
    end

    controller_handle = @updateAndControl;

    function u = updateAndControl(t_sample, y_measured)

if t_sample == 0
    p_x_hat      = zeros(6,1);
    % Initialise measured states directly from measurement
    % No need to estimate what we can directly observe
    p_x_hat(1)   = y_measured(1) - x_bar(1);   % x position deviation
    p_x_hat(3)   = y_measured(2) - x_bar(3);   % y position deviation
    p_x_hat(5)   = y_measured(3) - x_bar(5);   % angle deviation
    % Velocity states start at 0 (unknown, observer will estimate)
    p_x_hat(2)   = 0;
    p_x_hat(4)   = 0;
    p_x_hat(6)   = 0;
    p_z_int      = zeros(2,1);
    p_last_u     = u_bar;
end

        y_measured = y_measured(:);

        % 1. Observer update
        delta_u    = p_last_u - u_bar;
        delta_y    = y_measured - y_bar;
        p_x_hat    = (Gd - L*Cd) * p_x_hat + (Hd - L*Dd) * delta_u + L * delta_y;

        % 2. Integral update
        e       = Cr * (-p_x_hat);
        p_z_int = p_z_int + Ts * e;
        p_z_int = max(min(p_z_int, 10), -10);   % anti-windup

        % 3. Control law
        u = u_bar - Kd * p_x_hat - Ki * p_z_int;

        % 4. Force saturation
        u(1) = max(min(u(1),  40), -40);
        u(2) = max(min(u(2),  80),   0);

        p_last_u = u;
    end
end