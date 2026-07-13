%% Discrete-Time Control Synthesis: LQR & Observer Design
% Project: Airpong Levitation System
% Author: Lucas Ponchon

clear all; close all; clc;

%% 1. Parameters & System Identification
Ts = 0.1; 
% Transfer Function identified via System Identification Toolbox
num1 = [-0.031 0.07151];
den1 = [1 -1.802 0.802];
sys  = tf(num1, den1, Ts);
[A, B, C, D] = ssdata(sys);

%% 2. Augmented System for Integral Action
% Augmenting state-space with an integrator to reject disturbances
A_a = [A, zeros(2,1); -C, 1];
B_a = [B; 0];

%% 3. Controller Design (LQR)
% Q weights: [x1, x2, integral_error]. 
% Prioritizing integral error (Q(3,3)=5) for steady-state accuracy.
Q = [0.01 0 0; 0 0.1 0; 0 0 5]; 
R = 1; 
[K_a, ~, ~] = dlqr(A_a, B_a, Q, R);

K  = K_a(1:2); 
Ki = K_a(3);

%% 4. State Observer Design
% Luenberger observer: poles chosen 20% faster than controller poles
P_obs = eig(A - B*K) * 0.8; 
L = acker(A', C', P_obs)';

%% 5. Closed-Loop System Assembly (Global)
% Constructing the global system: [Plant, Observer, Controller]
A_glob = [ A,         -B*K,        -B*Ki;
           L*C,       A-L*C-B*K,   -B*Ki;
           -C,        zeros(1, 2),  1 ];

B_glob = [zeros(2,1); zeros(2,1); 1];
C_glob = [C, zeros(1, 2), 0];

%% 6. Validation & Visualization
sys_cl = ss(A_glob, B_glob, C_glob, 0, Ts);

figure('Name', 'Closed-Loop Performance');
step(sys_cl);
grid on;
title('Closed-loop step response (LQR + Observer)');
% saveas(gcf, 'step_response.png'); % Décommente pour sauvegarder le résultat
