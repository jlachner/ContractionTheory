clear; close all; clc;

%% Two stable linear systems \dot{x} = A x
A1 = [ 0   1;
      -2  -3];

A2 = [ 0   1;
      -5  -4];

%% 1) Euclidean contraction test
% Euclidean metric means M = I.
% Check whether A + A' is negative definite.

S1_euc = A1 + A1';
S2_euc = A2 + A2';

eig_S1_euc = eig(S1_euc)
eig_S2_euc = eig(S2_euc)

% If one eigenvalue is positive, the system is NOT contracting
% in the Euclidean metric.

%% 2) Construct contraction metrics using Lyapunov equation
% Find M such that:
%
% A' M + M A = -I
%
% If M is positive definite, then the system is contracting
% in the metric delta_x' M delta_x.

M1 = lyap(A1', eye(2));
M2 = lyap(A2', eye(2));

Q = diag([10 1]);
M1 = lyap(A1', Q);
M2 = lyap(A2', Q);

eig_M1 = eig(M1)
eig_M2 = eig(M2)

S1_metric = A1' * M1 + M1 * A1;
S2_metric = A2' * M2 + M2 * A2;

eig_S1_metric = eig(S1_metric)
eig_S2_metric = eig(S2_metric)


%% 3) Common metric for both systems

M_common = M1 + M2;

eig_M_common = eig(M_common)

S1_common = A1' * M_common + M_common * A1;
S2_common = A2' * M_common + M_common * A2;

eig_S1_common = eig(S1_common)
eig_S2_common = eig(S2_common)


%% 4) Blend the two systems

gain_arr = 0:0.1:1;

for gain = gain_arr

    A_blend = (1-gain)*A1 + gain*A2;

    S_blend_euc = A_blend + A_blend';
    S_blend_metric = A_blend' * M_common + M_common * A_blend;

    fprintf('\nGain = %.1f\n', gain);
    fprintf('Euclidean eig: ');
    fprintf('%.3f ', eig(S_blend_euc));
    fprintf('\n');

    fprintf('Metric eig:    ');
    fprintf('%.3f ', eig(S_blend_metric));
    fprintf('\n');

end