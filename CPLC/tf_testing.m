clear; clc; close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Define the System (Spring Mass Damper)

% Continuous Dynamics
k = 1;                          % spring constant
c = 0.01;                       % damping coefficient
A = [0  1; -k -c];              % continuous A matrix
B = [0; 1];                     % continuous B matrix
C = [-1 -c];                    % C matrix
D = 1;                          % D matrix

% Discretized Dynamics
fs = 32;                         % sampling frequency
dt = 1/fs;                      % samplinng time step
Ad = expm(A*dt);                % discrete A matrix
Bd = (Ad - eye(2))*A^-1*B;      % discrete B matrix

% Simulation Time
T0 = 0;                         % given initial time
Tf = 1024;                      % given final time (1024 samples)
T = T0:1/fs:(Tf - 1/fs);        % time values

% Frequencty Range (Nyquist Freq. = 1/2 Sampling Freq)
Np = length(T);
w = 0.5*fs*linspace(0,1,Np/2 + 1);  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Analytical Transfer Function 

% Analytical Solution to get |G(jw)| vs w
s = 1i*(2*pi)*w;                    % s = jw
G = 1./(s.^2 + c*s + k);            % G(jw)

mag_analytical = 20*log10(abs(G));  % |G(jw)| in dB

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Generate Sample Data

% Control Input
Ne = Np/4;
u = zeros(1,Np);
u(1:Ne)= randn(1,Ne).*hann(Ne).';   % u(t) = Random Force on Mass

% Simulate the System
x = [0; 0];
y = C*x + D*u(1);
for i = 2:length(T)
    x = Ad*x + Bd*u(i);
    y = [y (C*x + D*u(i))];         % y(t) = Acceleration of Mass
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Emperical G(jw) using u(t) and y(t) sample data

% Self-coded method to get |G(jw)| vs. w
U = fft(u, Np)/Np;                  % DFT of u(t) to get U(jw)
Y = fft(y, Np)/Np;                  % DFT of y(t) to get Y(jw)

YU = Y.*conj(U);                    % Cross-corellation function
UU = U.*conj(U);                    % Auto-corellation function
G1 = YU./UU;                         % Calculate G(jw)

mag_emperical_v1 = 20*log10(abs(G1));    % |G(jw)| using own code, DFTs 

% objective function
s = 1i*(2*pi)*w;
G = G(1:Np/2+1);
J = @(p) sum(1./(s.^2 + s.*p(1) + p(2)) - G).^2;

% perform optimization
p0 = [0.03, 1.02];         % initial guess for [c, k]
p_est = fminsearch(J, p0);
c_est = p_est(1);
k_est = p_est(2);

% estimated transfer function using fitted params
G_fit = 1 ./ (s.^2 + s*c_est + k_est);
mag_est_v1 = 20*log10(abs(G_fit));      % |G(jw)| using own code, approx

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Identified G(jw) using u(t) and y(t) sample data

% MATLAB function to get |G(jw)| vs. w 
G2 = tfestimate(u,y,Np,[],[], fs);
mag_emperical_v2 = 20*log10(abs(G2));         % |G(jw)| using tfestimate()

% MATLAB function to get G(s) = 1/(s^2 + cs + k)
data = iddata(y', u', dt);
tf_est = tfest(data, 2);                       % G(s) using tfest()
[G_est, ~] = bode(tf_est, (2*pi)*w);           % G(jw) using tfest()

mag_est_v2 = 20*log10(squeeze(G_est));              % |G(jw)| using tfest()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot the Data

% plot the sampled control
figure; 
subplot(2,1,1);
stairs(T,u, 'k'); xlim([T0, Tf]);
ylabel("$u(t)$", Interpreter='latex');
title("Control \& Acceleration vs. Time", Interpreter="latex")

% plot the sampled acceleration
subplot(2,1,2);
stairs(T, y, 'k'); xlim([T0, Tf])
ylabel("$\ddot{x}(t)$", Interpreter='latex');
xlabel("$t$ (s)", Interpreter="latex")

% plot the DFT magnitude
figure(); 
plot(w,mag_analytical, 'k', "DisplayName","Analytical G(s)"); hold on;
plot(w,mag_emperical_v1(1:Np/2+1), 'r-', "DisplayName","Emperical DFT - custom code" ); 
plot(w,mag_emperical_v2, 'b-', "DisplayName","Emperical DFT - tfestimate()"); 
plot(w,mag_est_v1, 'r--', "DisplayName","Identified G(s) - custom code" );
plot(w,mag_est_v2, 'b--', "DisplayName","Identified G(s) - tfest()"); 
ylim([-60 60]); 
xlim([w(1) 0.5]);
ylabel("$|G(j\omega)|$ (dB)", Interpreter="latex");
xlabel("$\omega$ (Hz)", Interpreter="latex")
grid on; 
title("$G(s)=\frac{1}{s^2+cs+k}$", Interpreter="latex")
legend();

% natural frequency
w_n = sqrt(k) / (2*pi);
disp(['w_n = ' num2str(w_n)]);