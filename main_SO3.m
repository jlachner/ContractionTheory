%% Initialization
clear; close all; clc;

% Change directory to the current .m location
cd( fileparts(matlab.desktop.editor.getActiveFilename) );

% Add path for functions
addpath( "utils", "DMP" )

% Add data for plotting
addpath( "GeometryLibrary/MATLAB" )
addpath( "draw_and_learn_trajectory/learned_parameters/discrete" )
addpath( "draw_and_learn_trajectory/learned_parameters/rhythmic" )

% Configure default figure properties
fig_config( 'fontSize', 20, 'markerSize', 10 )

colors = get(groot, 'defaultAxesColorOrder');

%% (Image, Type #1)
%% ===== (Image 4a) Sequential Combination of Movements, D-D-D, Minimum-jerk

data = load( 'min_jerk.mat' ); data = data.data;

% The number of basis functions, parameter are all identical
N = size( data.weight, 2 );

% The weights and goal location
% Since we are doing 3D, append the values
W = repmat( data.weight, 3, 1 ); g = repmat( data.goal, 3, 1 ); 

% The start and goal locations
start1 = [  0.0;  0.0; 0.0 ];  g1 = [  1.0;  0.8;  0.9 ]; 
start2 = [  0.5;  0.6; 0.7 ];  g2 = [ -0.3; -0.2;  0.5 ]; 
start3 = [ -0.5; -0.4; 0.6 ];  g3 = [  1.5; -1.5;  1.5 ]; 

% Other parameters are identical, hence calling from a single data set
tau_d = data.tau;
as    = data.alpha_s;
az    = data.alpha_z;
bz    = data.beta_z;

% Defining the DMPs
cs_d        = CanonicalSystem( 'discrete', tau_d, as );
trans_sys_d = TransformationSystem( az, bz, cs_d );
nft_d       = NonlinearForcingTerm( cs_d, N );

% The total time for the rollout
Td     = 10*tau_d;
Nd     = 10000;
td_arr = linspace( 0, Td, Nd+1 ); 

% Kinematic primitive inputs
prim_discrete1 = nft_d.calc_forcing_term( td_arr( 1:end-1 ), W, 0, diag( g1 - start1 ), 'trimmed' ) + az * bz * g1;
prim_discrete2 = nft_d.calc_forcing_term( td_arr( 1:end-1 ), W, 0, diag( g2 - start2 ), 'trimmed' ) + az * bz * g2;
prim_discrete3 = nft_d.calc_forcing_term( td_arr( 1:end-1 ), W, 0, diag( g3 - start3 ), 'trimmed' ) + az * bz * g3;

% Time shift of the discrete movement, 2nd and 3rd
idx1 = 1000; idx2 = 2000;
prim_discrete2_shifted = rightshift_with_wrap( prim_discrete2, idx1 );
prim_discrete3_shifted = rightshift_with_wrap( prim_discrete3, idx2 );

% Define the activation functions
basis_func1 = activation_func( Nd, idx1-5, idx1+5 );
basis_func2 = activation_func( Nd, idx2+5, idx2+15 );
act1_func =           1 - basis_func1;
act2_func = basis_func1 - basis_func2;
act3_func = basis_func2;

scl1 = 1.00; scl2 = 1.00; scl3 = 1.00;

prim_before_sum1 = scl1 * act1_func .* prim_discrete1;
prim_before_sum2 = scl2 * act2_func .* prim_discrete2_shifted;
prim_before_sum3 = scl3 * act3_func .* prim_discrete3_shifted;

prim_total = prim_before_sum1 + prim_before_sum2 + prim_before_sum3;

cl = [0 0.4470 0.7410];
f = figure( ); 
a1 = subplot( 1, 2, 1, 'parent', f );
hold on; axis equal;
drawsphere( a1 );
[ p_traj, ~, ~] = trans_sys_d.rollout( zeros( 3, 1 ), zeros( 3, 1 ), zeros( 3, 1 ), prim_total , 0, td_arr  ); 
plot3( a1, p_traj( 1, : ), p_traj( 2, : ), p_traj( 3, : ), 'color', 'k', 'linewidth', 3 )

% Overlap the original
[ p_traj1, ~, ~] = trans_sys_d.rollout( start1, zeros( 3, 1 ), zeros( 3, 1 ), prim_discrete1, 0, td_arr  ); 
[ p_traj2, ~, ~] = trans_sys_d.rollout( start2, zeros( 3, 1 ), zeros( 3, 1 ), prim_discrete2, 0, td_arr  ); 
[ p_traj3, ~, ~] = trans_sys_d.rollout( start3, zeros( 3, 1 ), zeros( 3, 1 ), prim_discrete3, 0, td_arr  ); 

draw3Dline( a1, p_traj1, ':', colors( 1, : ), 6, 10 );
draw3Dline( a1, p_traj2, ':', colors( 2, : ), 6, 10 );
draw3Dline( a1, p_traj3, ':', colors( 3, : ), 6, 10 );

limval = 4.0;
view( 3 ); set( a1, 'xlim', limval*[-1,1], 'ylim', limval*[-1,1], 'zlim', limval*[-1,1], 'xticklabel', {}, 'yticklabel', {}, 'zticklabel', {} )

a2 = subplot( 1, 2, 2, 'parent', f );
hold on; axis equal;

% The p_trajs are actually the exponential coordiantes of the frames. 
% Revive the SO3 matrices
R1 = zeros( 3, 3, Nd+1 ); R2 = zeros( 3, 3, Nd+1 ); R3 = zeros( 3, 3, Nd+1 );
R_total = zeros( 3, 3, Nd+1 );
for i = 1:Nd + 1
    R1( :, :, i ) = ExpSO3( R3_to_so3( p_traj1( :, i ) ) );
    R2( :, :, i ) = ExpSO3( R3_to_so3( p_traj2( :, i ) ) );
    R3( :, :, i ) = ExpSO3( R3_to_so3( p_traj3( :, i ) ) );
    R_total( :, :, i ) =  ExpSO3( R3_to_so3( p_traj( :, i ) ) );
end

draw3Dline( a2, p_traj1, '-', colors( 1, : ), 6, 10 );
draw3Dline( a2, p_traj2, '-', colors( 2, : ), 6, 10 );
draw3Dline( a2, p_traj3, '-', colors( 3, : ), 6, 10 );

mksize = 20;

for i = [ 100, 400, 500, 600, 900]
    drawframe( a2, p_traj( :, i ), R_total( :, :, i ), 0.2, 5, mksize );
end

for i = 1000+[ 100, 400, 500, 600, 900]
    drawframe( a2, p_traj( :, i ), R_total( :, :, i ), 0.2, 5, mksize );
end

for i = 2000+[ 100, 400, 500, 600, 900]
    drawframe( a2, p_traj( :, i ), R_total( :, :, i ), 0.2, 5, mksize );
end

% In between
for i = 900:20:1100
    drawframe( a2, p_traj( :, i ), R_total( :, :, i ), 0.2, 5, mksize );
end

for i = 1900:20:2100
    drawframe( a2, p_traj( :, i ), R_total( :, :, i ), 0.2, 5, 10 );
end

set( a2, 'view', [-37.5, 30], 'xticklabel', {}, 'yticklabel', {}, 'zticklabel', {} )

%% ===== (Image 4b) Parallel and Sequential Combination of Movements

data = load( 'min_jerk.mat' ); data = data.data;

% The number of basis functions, parameter are all identical
N = size( data.weight, 2 );

% The weights and goal location
% Since we are doing 3D, append the values
W = repmat( data.weight, 3, 1 ); g = repmat( data.goal, 3, 1 ); 

% The start and goal locations
start1 = [  0.0;  0.0; 0.0 ];  g1 = [  1.0;  0.8;  0.9 ]; 
start2 = [  0.5;  0.6; 0.7 ];  g2 = [ -0.3; -0.2;  0.5 ]; 
start3 = [ -0.5; -0.4; 0.6 ];  g3 = [  1.5; -1.5;  1.5 ]; 

% Other parameters are identical, hence calling from a single data set
tau_d = data.tau;
as    = data.alpha_s;
az    = data.alpha_z;
bz    = data.beta_z;

% Defining the DMPs
cs_d        = CanonicalSystem( 'discrete', tau_d, as );
trans_sys_d = TransformationSystem( az, bz, cs_d );
nft_d       = NonlinearForcingTerm( cs_d, N );

% The total time for the rollout
Td     = 10*tau_d;
Nd     = 10000;
td_arr = linspace( 0, Td, Nd+1 ); 

% Kinematic primitive inputs
prim_discrete1 = nft_d.calc_forcing_term( td_arr( 1:end-1 ), W, 0, diag( g1 - start1 ), 'trimmed' ) + az * bz * g1;
prim_discrete2 = nft_d.calc_forcing_term( td_arr( 1:end-1 ), W, 0, diag( g2 - start2 ), 'trimmed' ) + az * bz * g2;
prim_discrete3 = nft_d.calc_forcing_term( td_arr( 1:end-1 ), W, 0, diag( g3 - start3 ), 'trimmed' ) + az * bz * g3;

% Define the movement primitives, rhythmic
data = load( 'shaking.mat' ); data = data.data;

tau_r = data.tau;
Wr    = data.weight;

% Defining the DMPs
cs_r        = CanonicalSystem( 'rhythmic', tau_r, 1.0 );
trans_sys_r = TransformationSystem( az, bz, cs_r );
nft_r       = NonlinearForcingTerm( cs_r, N );

% Kinematic primitive inputs
prim_rhythmic = nft_r.calc_forcing_term( 6*td_arr( 1:end-1 ), Wr, 0, 0.6*eye( 3 ), 'trimmed' );

% Time shift of the discrete movement, 2nd and 3rd
idx1 = 1000; idx2 = 2000;
prim_discrete2_shifted = rightshift_with_wrap( prim_discrete2, idx1 );
prim_discrete3_shifted = rightshift_with_wrap( prim_discrete3, idx2 );

% Define the activation functions
basis_func1 = activation_func( Nd, idx1-5, idx1+5 );
basis_func2 = activation_func( Nd, idx2+5, idx2+15 );
act1_func =           1 - basis_func1;
act2_func = basis_func1 - basis_func2;
act3_func = basis_func2;

scl1 = 1.00; scl2 = 1.00; scl3 = 1.00;

prim_before_sum1 = scl1 * act1_func .* prim_discrete1;
prim_before_sum2 = scl2 * act2_func .* ( prim_discrete2_shifted + prim_rhythmic );
prim_before_sum3 = scl3 * act3_func .* prim_discrete3_shifted;

prim_total = prim_before_sum1 + prim_before_sum2 + prim_before_sum3;

cl = [0 0.4470 0.7410];
f = figure( ); 
a1 = subplot( 1, 2, 1, 'parent', f );
hold on; axis equal;
drawsphere( a1 );
[ p_traj, ~, ~] = trans_sys_d.rollout( zeros( 3, 1 ), zeros( 3, 1 ), zeros( 3, 1 ), prim_total , 0, td_arr  ); 
plot3( a1, p_traj( 1, : ), p_traj( 2, : ), p_traj( 3, : ), 'color', 'k', 'linewidth', 3 )

% Overlap the original
[ p_traj1, ~, ~] = trans_sys_d.rollout( start1, zeros( 3, 1 ), zeros( 3, 1 ), prim_discrete1, 0, td_arr  ); 
[ p_traj2, ~, ~] = trans_sys_d.rollout( start2, zeros( 3, 1 ), zeros( 3, 1 ), prim_discrete2, 0, td_arr  ); 
[ p_traj3, ~, ~] = trans_sys_d.rollout( start3, zeros( 3, 1 ), zeros( 3, 1 ), prim_discrete3, 0, td_arr  ); 

draw3Dline( a1, p_traj1, ':', colors( 1, : ), 6, 10 );
draw3Dline( a1, p_traj2, ':', colors( 2, : ), 6, 10 );
draw3Dline( a1, p_traj3, ':', colors( 3, : ), 6, 10 );

limval = 4.0;
view( 3 ); set( a1, 'xlim', limval*[-1,1], 'ylim', limval*[-1,1], 'zlim', limval*[-1,1], 'xticklabel', {}, 'yticklabel', {}, 'zticklabel', {} )

a2 = subplot( 1, 2, 2, 'parent', f );
hold on; axis equal;

% The p_trajs are actually the exponential coordiantes of the frames. 
% Revive the SO3 matrices
R1 = zeros( 3, 3, Nd+1 ); R2 = zeros( 3, 3, Nd+1 ); R3 = zeros( 3, 3, Nd+1 );
R_total = zeros( 3, 3, Nd+1 );
for i = 1:Nd + 1
    R1( :, :, i ) = ExpSO3( R3_to_so3( p_traj1( :, i ) ) );
    R2( :, :, i ) = ExpSO3( R3_to_so3( p_traj2( :, i ) ) );
    R3( :, :, i ) = ExpSO3( R3_to_so3( p_traj3( :, i ) ) );
    R_total( :, :, i ) =  ExpSO3( R3_to_so3( p_traj( :, i ) ) );
end

% draw3Dline( a2, p_traj1, '-', colors( 1, : ), 6, 10 );
draw3Dline( a2, p_traj2, ':', colors( 2, : ), 4, 10 );
% draw3Dline( a2, p_traj3, '-', colors( 3, : ), 6, 10 );

mksize = 20;

% for i = [ 100, 400, 500, 600, 900]
%     drawframe( a2, p_traj( :, i ), R_total( :, :, i ), 0.1, 5, 0.4*mksize );
% end

for i = 1100:50:1900
    drawframe( a2, p_traj( :, i ), R_total( :, :, i ), 0.2, 5, mksize );
end

% for i = 2000+[ 100, 400, 500, 600, 900]
%     drawframe( a2, p_traj( :, i ), R_total( :, :, i ), 0.1, 5, 0.4*mksize );
% end

set( a2, 'view', [-37.5, 30], 'xticklabel', {}, 'yticklabel', {}, 'zticklabel', {} )

%% (Image, Type #2)
%% ===== (Image 4a) Sequential Combination of Movements, D-D-D, Custom-drawn traj

data = load( 'hand_drawn.mat' ); data = data.data;

% The number of basis functions, parameter are all identical
N = size( data.weight, 2 );

% The weight and original goal
W = data.weight; g = data.goal;

% Other parameters are identical, hence calling from a single data set
tau_d = data.tau;
as    = data.alpha_s;
az    = data.alpha_z;
bz    = data.beta_z;

% Defining the DMPs
cs_d        = CanonicalSystem( 'discrete', tau_d, as );
trans_sys_d = TransformationSystem( az, bz, cs_d );
nft_d       = NonlinearForcingTerm( cs_d, N );

% The total time for the rollout
Td     = 5*tau_d;
Nd     = 10000;
td_arr = linspace( 0, Td, Nd+1 ); 

% Add offset, or starting position

% Kinematic primitive inputs
prim_discrete1 = nft_d.calc_forcing_term( td_arr( 1:end-1 ), W, 0, eye( 2 ), 'trimmed' ) + az * bz * g;
prim_discrete2 = nft_d.calc_forcing_term( td_arr( 1:end-1 ), W, 0, eye( 2 ), 'trimmed' ) + az * bz * g;
prim_discrete3 = nft_d.calc_forcing_term( td_arr( 1:end-1 ), W, 0, eye( 2 ), 'trimmed' ) + az * bz * g;

% Append a zero vector onto the prim_discrete1
prim_discrete1 = [ prim_discrete1; zeros( 1, size( prim_discrete1, 2 ) ) ];
prim_discrete2 = [ prim_discrete2; zeros( 1, size( prim_discrete2, 2 ) ) ];
prim_discrete3 = [ prim_discrete3; zeros( 1, size( prim_discrete3, 2 ) ) ];

% Get the scale and rotation matrix for the movements
scl1 = 0.01; scl2 = 0.01; scl3 = 0.01;

R1 = rotx(  30 ) * roty(  30 ) * rotz(  30 );
R2 = rotx( -30 ) * roty(  30 ) * rotz( -30 );
R3 = rotx(  30 ) * roty( -30 ) * rotz(  30 );

% Scale and rotate the movements
prim_discrete1 = scl1 * R1 * prim_discrete1;
prim_discrete2 = scl2 * R2 * prim_discrete2;
prim_discrete3 = scl3 * R3 * prim_discrete3;

% Time shift of the discrete movement, 2nd and 3rd
idx1 = 1000; idx2 = 2000;
prim_discrete2_shifted = rightshift_with_wrap( prim_discrete2, idx1 );
prim_discrete3_shifted = rightshift_with_wrap( prim_discrete3, idx2 );

% Define the activation functions
basis_func1 = activation_func( Nd, idx1-5, idx1+5 );
basis_func2 = activation_func( Nd, idx2+5, idx2+15 );
act1_func =           1 - basis_func1;
act2_func = basis_func1 - basis_func2;
act3_func = basis_func2;

prim_before_sum1 = act1_func .* prim_discrete1;
prim_before_sum2 = act2_func .* prim_discrete2_shifted;
prim_before_sum3 = act3_func .* prim_discrete3_shifted;

prim_total = prim_before_sum1 + prim_before_sum2 + prim_before_sum3;

cl = [0 0.4470 0.7410];

f = figure( ); 
a1 = subplot( 1, 2, 1, 'parent', f );
hold on; axis equal;
% drawsphere( a1 );
[ p_traj, ~, ~] = trans_sys_d.rollout( zeros( 3, 1 ), zeros( 3, 1 ), zeros( 3, 1 ), prim_total , 0, td_arr  ); 
plot3( a1, p_traj( 1, : ), p_traj( 2, : ), p_traj( 3, : ), 'color', 'k', 'linewidth', 3 )

% Overlap the original
[ p_traj1, ~, ~] = trans_sys_d.rollout( zeros( 3, 1 ), zeros( 3, 1 ), zeros( 3, 1 ), prim_discrete1, 0, td_arr  ); 
[ p_traj2, ~, ~] = trans_sys_d.rollout( zeros( 3, 1 ), zeros( 3, 1 ), zeros( 3, 1 ), prim_discrete2, 0, td_arr  ); 
[ p_traj3, ~, ~] = trans_sys_d.rollout( zeros( 3, 1 ), zeros( 3, 1 ), zeros( 3, 1 ), prim_discrete3, 0, td_arr  ); 

draw3Dline( a1, p_traj1, ':', colors( 1, : ), 6, 10 );
draw3Dline( a1, p_traj2, ':', colors( 2, : ), 6, 10 );
draw3Dline( a1, p_traj3, ':', colors( 3, : ), 6, 10 );

view( 3 );
