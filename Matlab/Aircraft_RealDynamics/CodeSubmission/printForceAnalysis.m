function printForceAnalysis(u_sim0, u_sim, x0, x_bar, Kd, u_bar, label)
% printForceAnalysis  Prints actuator force usage and constraint checking
%
%   This function evaluates actuator effort for a control system under:
%   - No disturbance
%   - With disturbance
%
%   It also checks whether actuator limits are violated.
%
% INPUTS:
%   u_sim0 : Control inputs (no disturbance case)
%   u_sim  : Control inputs (disturbance case)
%   x0     : Initial state vector
%   x_bar  : Equilibrium state vector
%   Kd     : State feedback gain matrix
%   u_bar  : Equilibrium input vector
%   label  : String label for display (e.g. 'Observer Controller')
%
% OUTPUTS:
%   (none) - results printed to command window
%
% NOTES:
%   - Computes initial control effort from state deviation
%   - Checks actuator saturation limits:
%       F1: ±40 N
%       F2: 0–80 N
%   - Reports max, min, and average usage

fprintf('\n=== Force Analysis - %s ===\n', label)
fprintf('F1 at t=0: %.2f N\n', u_sim0(1,1))
fprintf('F2 at t=0: %.2f N\n', u_sim0(1,2))

fprintf('\n--- No Disturbance ---\n')
fprintf('Max F1 = %.2fN (limit ±40N)\n', max(abs(u_sim0(:,1))))
fprintf('Max F2 = %.2fN (limit 0-80N)\n', max(u_sim0(:,2)))
fprintf('Avg F1 = %.2fN\n', mean(abs(u_sim0(:,1))))
fprintf('Avg F2 = %.2fN\n', mean(u_sim0(:,2)))
fprintf('Min F2 = %.2fN\n', min(u_sim0(:,2)))

fprintf('\n--- With Disturbance ---\n')
fprintf('Max F1 = %.2fN (limit ±40N)\n', max(abs(u_sim(:,1))))
fprintf('Max F2 = %.2fN (limit 0-80N)\n', max(u_sim(:,2)))
fprintf('Avg F1 = %.2fN\n', mean(abs(u_sim(:,1))))
fprintf('Avg F2 = %.2fN\n', mean(u_sim(:,2)))
fprintf('Min F2 = %.2fN\n', min(u_sim(:,2)))

fprintf('\n--- Limit Compliance ---\n')
if max(abs(u_sim0(:,1))) <= 40 && max(abs(u_sim(:,1))) <= 40
    fprintf('F1: COMPLIANT (both cases within ±40N)\n')
else
    fprintf('F1: VIOLATION detected\n')
end
if min(u_sim0(:,2)) >= 0 && max(u_sim0(:,2)) <= 80 && ...
   min(u_sim(:,2))  >= 0 && max(u_sim(:,2))  <= 80
    fprintf('F2: COMPLIANT (both cases within 0-80N)\n')
else
    fprintf('F2: VIOLATION detected\n')
end
end