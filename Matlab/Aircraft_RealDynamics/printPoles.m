function printPoles(G, H, K, label)
% printPoles  Prints pole locations and stability of a discrete system
%
%   This function computes and displays the poles of either:
%   - an open-loop system, or
%   - a closed-loop state feedback system
%
%   It also provides a basic stability assessment based on
%   the unit circle criterion (discrete-time systems).
%
% INPUTS:
%   G     : State matrix (A matrix in state-space form)
%   H     : Input matrix (B matrix in state-space form)
%   K     : State feedback gain matrix
%           - If empty [], system is treated as open-loop
%           - If provided, closed-loop poles are computed as (G - H*K)
%   label : String label for display (e.g. 'Open Loop', 'Closed Loop')
%
% OUTPUTS:
%   (none) - results are printed to the command window

    if isempty(K)
        poles = eig(G);        % open loop
    else
        poles = eig(G - H*K);  % closed loop
    end

    fprintf('\n=== %s Poles ===\n', label)

    % Stability assessment
    if any(abs(poles) > 1)
        fprintf('Stability: UNSTABLE (poles outside unit circle)\n')
    elseif any(abs(poles) == 1)
        fprintf('Stability: MARGINALLY STABLE (poles on unit circle)\n')
    else
        fprintf('Stability: STABLE (all poles inside unit circle)\n')
    end

    fprintf('Slowest pole: %.6f\n', max(abs(poles)))
    fprintf('Fastest pole: %.6f\n', min(abs(poles)))
    fprintf('All poles (sorted slow to fast):\n')
    sorted = sort(abs(poles), 'descend');
    for i = 1:length(sorted)
        fprintf('  Pole %d: %.6f\n', i, sorted(i))
    end
end