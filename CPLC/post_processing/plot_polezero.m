function plot_polezero(body, ctrl)
%POLES PLOTS POLE ZERO MAP OF OPEN LOOP AND CLOSED-LOOP SYSTEM

A = body.A;
B = body.B;

C = eye(height(A));                    % Estimator will provide full state
D = zeros(height(C),width(B));         % D matrix is always zero for LQG

K = ctrl.K;
T = ctrl.T;

% OPEN-LOOP SYSTEM
sys1 = ss(A, B, C, D);

% CLOSED-LOOP SYSTEM
[phi, gamma] = c2d(A, B, T);
sys2 = ss(phi-gamma*K, 0*B, C-D*K, 0*D, T);

% PLOTS
figure();
pzmap(sys1); title("Open Loop Pole-Zero Map");
figure();
pzmap(sys2); title("Closed Loop Pole-Zero Map")
end

