%% Initialization 
clear all; close all; clc;

% Change the directory to the current directory 
cd( fileparts( matlab.desktop.editor.getActiveFilename ) );

% Add path for functions, which are one directory up
addpath( "../utils", "../DMP" )

%% (Section 1) Imitation Learning, Discrete Movement minimum-jerk trajectory
%% ----------- (1A) Load Data 

% Call the data
traj_name = 'min_jerk';

% The time length of the trajectory
T = 1;
P = 100;
pi = 0; pf = 1;

% Parse data
t_arr   =   linspace( 0, 1, P );
p_arr   =  pi + ( pf - pi ) * ( 10 * t_arr.^3 -  15 * t_arr.^4 +   6 * t_arr.^5 ) ;
dp_arr  =       ( pf - pi ) * ( 30 * t_arr.^2 -  60 * t_arr.^3 +  30 * t_arr.^4 ) ;
ddp_arr =       ( pf - pi ) * ( 60 * t_arr.^1 - 180 * t_arr.^2 + 120 * t_arr.^3 ) ;

% Adding an offset to make the initial position 0
% Although this is already done during the drawing data process,
% We again do this just to double check 
p_arr = p_arr - p_arr( :, 1 );

P = length( p_arr );    % Length of the data
N = 50;                 % The number of basis functions

alpha_s = 1.0;
alpha_z = 100.0;
beta_z  = 0.25 * alpha_z;
tau_d = max( t_arr );

% Defining the DMPs
cs_d        = CanonicalSystem( 'discrete', tau_d, alpha_s );
trans_sys_d = TransformationSystem( alpha_z, beta_z, cs_d );
nft_d       = NonlinearForcingTerm( cs_d, N );

% For discrete movement, goal is the final trajectory
gd = p_arr( :, end );

% Calculate the B matrix
B_arr = trans_sys_d.get_desired( p_arr, dp_arr, ddp_arr, gd );

% The A Matrix
A_arr = zeros( N, P );

% Calculate the Phi matrix for Weight Learning
% Note that compared to Ijspeert 2013, this followed Koutras and Doulgeri (2020)
for i = 1 : P 
    t = t_arr( i );
    A_arr( :, i ) = ( nft_d.calc_multiple_ith( t, 1:N )/ nft_d.calc_whole_at_t( t ) )*cs_d.calc( t );
end

% Learning the weights with Linear Least-square fitting
w_arr = B_arr * A_arr' * inv( A_arr * A_arr' );

%% ----------- (1B) Double Check the learning result

% The total time for the rollout
Tr     = 2*tau_d;
Nr     = 10000;
tr_arr = linspace( 0, Tr, Nr+1 ); 

% Kinematic primitive input
kinematic_prim = nft_d.calc_forcing_term( tr_arr( 1:end-1 ), w_arr, 0, 1 ) + alpha_z*beta_z*gd;
[ p_roll, ~, ~] = trans_sys_d.rollout( 0, 0, 0, kinematic_prim, 0, tr_arr  ); 

f = figure( ); a = axes( 'parent', f );
hold( a, 'on' ); axis equal

plot( a, p_roll, p_roll, 'linewidth', 4 )
plot( a,  p_arr,  p_arr, 'linewidth', 8, 'linestyle', '--' , 'color', 'k')

% If data looks correct, save the raw data 

data = struct;

% Parameters of DMP and Learned Weighted
data.name    = traj_name;
data.tau     = tau_d;
data.alpha_s = alpha_s;
data.alpha_z = alpha_z;
data.beta_z  =  beta_z;
data.weight  =  w_arr;

% Goal Location, where initial position is automatically zero
data.goal =  gd;

save( ['learned_parameters/discrete/', traj_name,'.mat' ], 'data' );

