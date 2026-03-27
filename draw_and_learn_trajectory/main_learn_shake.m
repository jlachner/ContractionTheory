%% Initialization 
clear all; close all; clc;

% Change the directory to the current directory 
cd( fileparts( matlab.desktop.editor.getActiveFilename ) );

% Add path for functions, which are one directory up
addpath( "../utils", "../DMP", "../GeometryLibrary/MATLAB" )

% Add path where the raw data is saved
addpath( "./raw_data/rhythmic" )

%% (Section 1) Imitation Learning, Rhythmic Movement shaking motion
%% ----------- (1A) Load Data 

load( 'rot_shake.mat' )

% Get the number of data
P = size( R_shake, 3 );

% Get the logarithmic map of this trajectory
e_arr = zeros( 3, P );

for i = 1 : P
    e_arr( :, i ) = so3_to_R3( LogSO3( R_shake( :, :, i ) ) );
end

% Compensate for the offset
e1 = e_arr( :, 1 ); e2 = e_arr( :, end );
t_arr = linspace( 0, tau, P );
e_compen = ( e2 - e1 )/max( tau ) * t_arr + e1;

% Taking off from e_arr
e_arr = e_arr - e_compen;

f = figure( ); a = axes( 'parent', f ); 
hold on; axis equal;
drawsphere( a );

plot3( a, e_arr( 1, : ), e_arr( 2, : ), e_arr( 3, : ), 'linewidth', 3, 'color', 'k' )

limval = 4.0;
view( 3 ); set( a, 'xlim', limval*[-1,1], 'ylim', limval*[-1,1], 'zlim', limval*[-1,1], 'visible', 'off' )

% Also plot the de, dde
[  de_arr,  de_arr_smth ] = diff_smooth( e_arr, t_arr, 100 );
[ dde_arr, dde_arr_smth ] = diff_smooth( de_arr_smth, t_arr, 100 );

%% Learning the weights

alpha_s = 1.0;
alpha_z = 100.0;
beta_z  = 0.25 * alpha_z;
tau_r = max( t_arr )/(2*pi);

N = 50;

% Defining the DMPs
cs_r        = CanonicalSystem( 'rhythmic', tau_r, alpha_s );
trans_sys_r = TransformationSystem( alpha_z, beta_z, cs_r );
nft_r       = NonlinearForcingTerm( cs_r, N );

% Calculating the B matrix (or array)
B_mat = trans_sys_r.get_desired( e_arr, de_arr_smth, dde_arr_smth, mean( e_arr, 2 ) );

% The phi matrix 
A_mat = zeros( N, P );

% Calculate the Phi matrix for Weight Learning
% Note that compared to Ijspeert 2013, this followed Koutras and Doulgeri (2020)
for i = 1 : P 
    t = t_arr( i );
    A_mat( :, i ) = nft_r.calc_multiple_ith( t, 1:N )/ nft_r.calc_whole_at_t( t );
end

% Learning the weights with Linear Least-square fitting
weight = B_mat * A_mat' * inv( A_mat * A_mat' );

%% Rollout and compare 

% Actual rollout 
t0i   = 0.0;
dt    = 1e-3;
t_rollout = 0:dt:(1.0*tau*5);
Nt    = length( t_arr );

% The scaling factor 
scl = 1.0;
input_arr = nft_r.calc_forcing_term( t_rollout( 1:end-1 ), weight, t0i, eye( 3 ) );

[ e_rollout, ~, ~, ] = trans_sys_r.rollout( zeros( 3, 1 ), zeros( 3, 1 ), scl*mean( e_arr, 2 ), scl*input_arr, t0i, t_rollout ); 

% Plot the difference between 
f = figure( ); hold on;
plot(  t_rollout, e_rollout, 'linewidth', 3, 'color', 'k' );
plot(  t_arr,  e_arr, 'linewidth', 5, 'color', 'r', 'linestyle', '--' );

R_arr = zeros( 3, 3, Nt );

for i = 1 : Nt
    R_arr( :, :, i ) = ExpSO3( R3_to_so3( e_arr( :, i ) ) )';
end

% Reshaping A into a 3x(3*N) array
R_arr_save = reshape(R_arr, 3, []);

% The data structure
data = struct;

% Parameters of DMP and Learned Weighted
traj_name    = 'shaking';
data.name    = traj_name;
data.tau     = tau_r;
data.alpha_z = alpha_z;
data.beta_z  =  beta_z;
data.weight  =  weight;

% Goal Location, where initial position is automatically zero
data.goal = mean( e_arr, 2 );

save( ['learned_parameters/rhythmic/', traj_name,'.mat' ], 'data' );
