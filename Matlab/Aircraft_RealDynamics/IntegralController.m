function Controller = IntegralController(Kd, Ki, x_ref, u_eq, Cr, Ts)
% INTEGRALCONTROLLER  State feedback controller with integral action
%
% DESCRIPTION:
%   Implements full-state feedback with integral action for tracking:
%       u = -Kd*(x - x_ref) - Ki*integral(error) + u_eq
%
% INPUTS:
%   Kd    - State feedback gain matrix
%   Ki    - Integral gain matrix
%   x_ref - Reference state vector
%   u_eq  - Equilibrium input
%   Cr    - Output selection matrix
%   Ts    - Sampling time
%
% OUTPUT:
%   Controller - function handle for simulation

    % store accumulated integral error
    persistent z_int

    % Initialise integral state
    z_int = zeros(2,1);

    % Return controller function handle
    Controller = @control_law;
    
    %   u = -Kd*x_error - Ki*integral_error + u_eq
    function u = control_law(t, x) % computes actuator forces at each timestep

        if t == 0
            z_int = zeros(2,1);
        end

        % State deviation from desired reference
        delta_x = x - x_ref;

        % Integral update
        e = Cr * (x_ref - x);      % Compute tracking error from reference
        
        % X: conditional integration — prevents windup during large horizontal approach
        if abs(e(1)) < 2.0
            z_int(1) = z_int(1) + Ts * e(1);   % Forward Euler integration
        else
            z_int(1) = z_int(1) * 0.95;        % Slowly decay integral when far away
        end

        % Y: conditional integration with larger threshold — needs more authority for disturbance rejection
        if abs(e(2)) < 4.0
            z_int(2) = z_int(2) + Ts * e(2);   % Forward Euler integration
        else
            z_int(2) = z_int(2) * 0.95;        % Slowly decay integral when far away
        end

        % Anti-windup based on actuator headroom
        u_headroom = [35; 35];                            % Integral contribution cap
        z_limit    = abs(Ki) \ u_headroom;                % Convert allowable control effort into equivalent integrator state limits
        z_int      = max(min(z_int, z_limit), -z_limit);  % Anti-windup protection

        % Full controller equation
        u = -Kd*delta_x - Ki*z_int + u_eq;

        % Force limits
        u(1) = max(min(u(1), 40), -40); % F1 Limited to +/- 40N
        u(2) = max(min(u(2), 80), 0); % F2 Limited to 0 - 80N
    end
end