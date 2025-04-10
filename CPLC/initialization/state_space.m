function [A, B] = state_space(body, act)
% STATE_SPACE.M OUTPUTS CONTINUOUS STATE SPACE MODEL
%   x = [phi theta psi u v w p q r]
%   u = [u_1 u_2 ... u_m]
%   dx = Ax + Bu
%   y = Cx + Du

% USER INPUT
phi0 = body.x_bar(1);
theta0 = body.x_bar(2);
psi0 = body.x_bar(3);
u0 = body.x_bar(4);
v0 = body.x_bar(5);
w0 = body.x_bar(6);
p0 = body.x_bar(7);
q0 = body.x_bar(8);
r0 = body.x_bar(9);
m = body.m;
I = body.I;
lx = act.lx;
ly = act.ly;
tc1 = act.tau_engine;
tc2 = act.tau_rcs;
rcs = act.rcs;

% DIMENSIONS
nx = 9;                  % number of states
nu = length(rcs) + 1;    % number of controls

%% A MATRIX

% SYMBOLIC TERMS
syms p q r phi theta;
psi = sym('psi');       % this is needed specifically for some reason

% EOMS: {dtheta, dphi, dpsi, du, dv, dw} wrt {p, q, r, u, v, w}
A =[[0 0 0  0   0   0   1   0   0];  % dphi = p
    [0 0 0  0   0   0   0   1   0];  % dtheta = q
    [0 0 0  0   0   0   0   0   1];  % dpsi = r
    [0 0 0  0   r0 -q0  0  -w0  v0]; % du = r0*v-q0*w-w0*q+v0*r
    [0 0 0 -r0  0   p0  w0  0  -u0]; % dv =-r0*u+p0*w+w0*p-u0*r
    [0 0 0  q0 -p0  0  -v0  u0  0];  % dw = q0*u-p0*v-v0*p+u0*q 
    [0 0 0  0   0   0   0   0   0];  % ~
    [0 0 0  0   0   0   0   0   0];  % ~
    [0 0 0  0   0   0   0   0   0]]; % ~

% EOMS: {du, dv dw} wrt {phi, theta, psi}
T1 = [1  0        0       ;          % rotation about x 
      0  cos(phi) sin(phi); 
      0 -sin(phi) cos(phi);];
T2 = [cos(theta) 0 -sin(theta);      % rotation about y
      0          1  0         ; 
      sin(theta) 0  cos(theta);];
T3 = [ cos(psi) sin(psi) 0;
      -sin(psi) cos(psi) 0; 
       0        0        1;];        % rotation about z
g = T1*T2*T3*[-9.81; 0; 0];          % g_body = R(3,2,1)*g_inertial
dg = [diff(g, phi) diff(g, theta) diff(g, psi)];         % taylor series
dg0 = subs(dg, {phi, theta, psi}, {phi0, theta0, psi0}); % taylor series
A(4:6, 1:3) = dg0;                                       % add to A

% EOMS: {dp, dq, dr} wrt {p, q, r}
wIw = cross([p; q; r], I*[p; q; r]);                % I*dw = w x (I*w)
dwIw = [diff(wIw, p) diff(wIw, q) diff(wIw, r)];    % taylor series
dwIw0 = subs(dwIw, {p, q, r}, {p0, q0, r0});        % taylor series
A(7:9,7:9) = (I^-1)*dwIw0;                          % add to A

%% B MATRIX

B = zeros(nx, nu);                        % initialize B matrix
B(:,1) = [0; 0; 0; 1; 0; 0; 0; 0; 0];     % add engine force
for i = 2:(width(B))
    % add forces & moments for each rcs thrusters
    if rcs(i - 1) == 1
        B(:, i) = [0; 0; 0; 0; 1; 0; 0; 0; lx];     % pair 1
    elseif rcs(i - 1) == 2
        B(:, i) = [0; 0; 0; 0; 0; 1; -ly; -lx; 0];  % pair 2
    elseif rcs(i - 1) == 3
        B(:, i) = [0; 0; 0; 0; 0; 1; ly; -lx; 0];   % pair 3
    elseif rcs(i - 1) == 4
        B(:, i) = [0; 0; 0; 1; 0; 0; 0; 0; ly];     % pair 4
    elseif rcs(i - 1) == 5
        B(:, i) = [0; 0; 0; 1; 0; 0; 0; 0; -ly];    % pair 5
    end
end
B(1:6,:) = B(1:6,:)/m;       % convert forces to tranlational acceleration
B(7:9,:) = (I^-1)*B(7:9,:);  % convert moments to rotational acceleration

%% ACTUATOR DYNAMICS

% AUGMENTED A MATRIX
A_UL = A;                               % old A matrix (upper left)
A_UR = B;                               % old B matrix (upper right)
A_LL = zeros(nu, nx);                   % zeros of thrusters (lower left)
A_LR = eye(nu, nu);                     % actuator dynamics (lower right)
A_LR(1,1) = -tc1;                       % add the engine time constant
A_LR(2:nu, 2:nu) = -A_LR(2:nu, 2:nu)*tc2; % add the rcs time constant
A = [A_UL A_UR; A_LL A_LR];             % make the new A matrix
 
% AUGMENTED B MATRIX
B_U = zeros(nx, nu);                    % zeros for the states (upper)
B_L = eye(nu, nu);                      % I of the thrusters (lower)
B_L(1,1) = tc1;                         % add the engine time constant
B_L(2:nu, 2:nu) = B_L(2:nu, 2:nu)*tc2;  % add the rcs time constant
B = [B_U; B_L];                         % make the new B matrix
end

