function controller_handle = ObserverIntegralController(Gd, Hd, Cd, Dd, L, Kd, Ki, Cr, x_bar, u_bar, Ts)
% ObserverIntegralController  Observer-based state feedback controller with integral action
%
%   This function implements a discrete-time controller that combines:
%   - Luenberger state observer
%   - State feedback control (Kd)
%   - Integral action (Ki)
%   - Actuator saturation limits
%
% INPUTS:
%   Gd     : Discrete-time state matrix (A matrix)
%   Hd     : Discrete-time input matrix (B matrix)
%   Cd     : Output matrix
%   Dd     : Feedthrough matrix
%   L      : Observer gain matrix
%   Kd     : State feedback gain matrix
%   Ki     : Integral gain matrix
%   Cr     : Output selection matrix (tracks position states)
%   x_bar  : Equilibrium state vector
%   u_bar  : Equilibrium input vector
%   Ts     : Sampling time (seconds)
%
% OUTPUT:
%   controller_handle : Function handle used by simulator
%
% INTERNAL STATES:
%   p_x_hat   : Estimated state deviation
%   p_z_int   : Integral tracking error state
%   p_last_u  : Previous control input (used in observer update)

    persistent p_x_hat p_z_int p_last_u p_initialized

    x_bar = x_bar(:);
    u_bar = u_bar(:);
    y_bar = Cd * x_bar;   % = [12; 12; 0]

    if isempty(p_initialized)

        %Initial States
        p_x_hat       = zeros(6,1);
        p_z_int       = zeros(2,1);
        p_last_u      = u_bar;
        p_initialized = true;
    end

    controller_handle = @updateAndControl;

    function u = updateAndControl(t_sample, y_measured)

 if t_sample == 0
    y_measured = y_measured(:);
    
    % Initialise position states from direct measurement
    % This prevents false velocity estimates from large initial position error
    p_x_hat    = zeros(6,1);
    p_x_hat(1) = y_measured(1) - x_bar(1);   % x position deviation  = -12
    p_x_hat(3) = y_measured(2) - x_bar(3);   % y position deviation  = -12
    p_x_hat(5) = y_measured(3) - x_bar(5);   % angle deviation       =  0
    % Velocities unknown — start at 0, observer will estimate
    p_x_hat(2) = 0;                          % X velocity
    p_x_hat(4) = 0;                          % Y velocity
    p_x_hat(6) = 0;                          % Angular Rate
    p_z_int    = zeros(2,1);
    p_last_u   = u_bar;
end

        y_measured = y_measured(:);

        % 1. Observer update
        delta_u    = p_last_u - u_bar;      % Convert prev control input into deviation coordinates
        delta_y    = y_measured - y_bar;    % Difference between measured output and equilibrium output
        p_x_hat    = (Gd - L*Cd) * p_x_hat + (Hd - L*Dd) * delta_u + L * delta_y;

        % 2. Integral update
        e = Cr * (-p_x_hat);        % Compute tracking error from reference
        
        % X: conditional integration — prevents windup during large horizontal approach
        if abs(e(1)) < 2.0
            p_z_int(1) = p_z_int(1) + Ts * e(1);    % Forward Euler integration
        else
            p_z_int(1) = p_z_int(1) * 0.95;         % Slowly decay integral when far away
        end
        
        % Y: conditional integration with larger threshold — needs more authority for disturbance rejection
        if abs(e(2)) < 4.0         
            p_z_int(2) = p_z_int(2) + Ts * e(2);    % Forward Euler integration
        else  
            p_z_int(2) = p_z_int(2) * 0.95;         % Slowly decay integral when far away
        end
        
        % Anti-windup based on actuator headroom
        u_headroom = [35; 35];                                  % Integral contribution cap
        z_limit    = abs(Ki) \ u_headroom;                      % Convert allowable control effort into equivalent integrator state limits
        p_z_int    = max(min(p_z_int, z_limit), -z_limit);      % Anti-windup protection

        % 3. Observer-based state feedback with integral action (Control law)
        u = u_bar - Kd * p_x_hat - Ki * p_z_int;

        % 4. Actuator limits 
        u(1) = max(min(u(1),  40), -40); % Horizontal limit +- 40N
        u(2) = max(min(u(2),  80),   0); % Vertical Limit 0 - 80N  

        p_last_u = u;
    end
end