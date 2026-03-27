%% Initialization
clear all; close all; clc;

% Change directory to the current .m location
cd( fileparts(matlab.desktop.editor.getActiveFilename) );

% Add path for functions
addpath( "utils", "DMP" )

% Add data for plotting
addpath( "draw_and_learn_trajectory/learned_parameters/rhythmic" )
addpath( "draw_and_learn_trajectory/learned_parameters/discrete" )

% Configure default figure properties
fig_config( 'fontSize', 20, 'markerSize', 10 )

%% (Image 1a) Dynamic Movement Primitives, from the Definition of kinematic primitives

% Import both discrete and rhythmic movement primitives
% ================================================= %
% =================== Discrete ==================== %
% ================================================= %
data_d = load( 'A.mat'     ); data_d = data_d.data;

N = size( data_d.weight, 2 );

% Defining the DMPs
cs_d        = CanonicalSystem( 'discrete', data_d.tau, data_d.alpha_s );
trans_sys_d = TransformationSystem( data_d.alpha_z, data_d.beta_z, cs_d );
nft_d       = NonlinearForcingTerm( cs_d, N );
gd          = data_d.goal;

% The total time for the rollout
Tr     = 2*data_d.tau;
Nr     = 10000;
tr_arr = linspace( 0, Tr, Nr+1 ); 

% Kinematic primitive input
kinematic_prim_discrete = nft_d.calc_forcing_term( tr_arr( 1:end-1 ), data_d.weight, 0, eye( 2 ), 'trimmed' ) + data_d.alpha_z*data_d.beta_z*gd;
[ p_roll_d, ~, ~] = trans_sys_d.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), kinematic_prim_discrete, 0, tr_arr  ); 

% ================================================= %
% =================== Rhythmic ==================== %
% ================================================= %
data_r = load( 'heart.mat' ); data_r = data_r.data;

N = size( data_d.weight, 2 );

% Defining the DMPs
cs_r        = CanonicalSystem( 'rhythmic', data_r.tau, 1.0 );
trans_sys_r = TransformationSystem( data_r.alpha_z, data_r.beta_z, cs_r );
nft_r       = NonlinearForcingTerm( cs_r, N );
gr          = data_r.goal;

% The total time for the rollout
Tr     = 2*(2*pi*data_r.tau);
Nr     = 10000;
tr_arr = linspace( 0, Tr, Nr+1 ); 

% Kinematic primitive input
kinematic_prim_rhythmic = nft_r.calc_forcing_term( tr_arr( 1:end-1 ), data_r.weight, 0, eye( 2 ) ) + data_r.alpha_z*data_r.beta_z*gr;
[ p_roll_r, ~, ~] = trans_sys_r.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), kinematic_prim_rhythmic, 0, tr_arr  ); 

f1 = figure( ); a1 = axes( 'parent', f1 );
hold on
axis( a1, 'equal' )
plot( a1, p_roll_d( 1, : ), p_roll_d( 2, : ), 'linewidth', 5, 'color', [0 0.4470 0.7410] )
scatter( a1, p_roll_d( 1, 1 ), p_roll_d( 2, 1 ),  400, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', [0 0.4470 0.7410], 'linewidth', 5 )
scatter( a1, p_roll_d( 1, end ), p_roll_d( 2, end ), 400, 'd','filled', 'markerfacecolor', 'w', 'markeredgecolor', [0 0.4470 0.7410], 'linewidth', 5 )
set( a1, 'xticklabel', {}, 'yticklabel', {}, 'xlim', [-2, 8], 'ylim', [-1,12] )
exportgraphics( f1, 'images/fig1a_1.pdf', 'ContentType', 'vector');

f2 = figure( ); a2 = axes( 'parent', f2 );
hold on
axis( a2, 'equal' )
plot( a2, p_roll_r( 1, : ), p_roll_r( 2, : ), 'linewidth', 5, 'color', [0.8500 0.3250 0.0980]	 )
scatter( a2, p_roll_r( 1, 1 ), p_roll_r( 2, 1 ),  400, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', [0.8500 0.3250 0.0980], 'linewidth', 5 )
set( a2, 'xticklabel', {}, 'yticklabel', {}, 'xlim', [-20, 20], 'ylim', [-25, 10] )
exportgraphics( f2, 'images/fig1a_2.pdf', 'ContentType', 'vector');

%% (Image 1b) Dynamic Movement Primitives, Scaling Property 

f1 = figure( ); a1 = axes( 'parent', f1 );
hold on
axis( a1, 'equal' )

% The total time for the rollout
Tr     = 2*data_d.tau;
Nr     = 10000;
tr_arr = linspace( 0, Tr, Nr+1 ); 

spc = [ 1.0, 1.5, 2.0, 2.6, 4.2, 6.5, 8.0, 11.5 ];
spc = cumsum( spc );
i = 1.0;

for kappa = [ 0.1, 0.2, 0.3, 0.5, 0.8, 1.0, 1.5, 2.0 ]
    % Kinematic primitive input
    [ p_roll_d, ~, ~] = trans_sys_d.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), kappa*kinematic_prim_discrete, 0, tr_arr  ); 
    if kappa == 1.0
        lw = 5;
        cl = [0 0.4470 0.7410];
    else
        lw = 2.4;
        cl = 0.3*ones( 1, 3 );
    end

    plot( a1, spc( i ) + p_roll_d( 1, : ), p_roll_d( 2, : ), 'linewidth', lw, 'color', cl )
    scatter( a1, spc( i ) + p_roll_d( 1, end ), p_roll_d( 2, end ), kappa*300, 'd','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth', kappa*2 )
    scatter( a1, spc( i ) + p_roll_d( 1, 1 ), p_roll_d( 2, 1 ),  kappa*150, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth', kappa*2 )

    i = i + 1;
end
set( a1, 'xticklabel', {}, 'yticklabel', {}, 'xlim', [ 0.0, 52 ], 'ylim', [-5, 25] )
exportgraphics( f1, 'images/fig1b_1.pdf', 'ContentType', 'vector');

f2 = figure( ); a2 = axes( 'parent', f2 );
hold on
axis( a2, 'equal' )

% The total time for the rollout
Tr     = 2*(2*pi*data_r.tau);
Nr     = 10000;
tr_arr = linspace( 0, Tr, Nr+1 ); 

for kappa = [ 0.1, 0.2, 0.3, 0.5, 0.8, 1.0, 1.5, 2.0 ]
    % Kinematic primitive input
    [ p_roll_r, ~, ~] = trans_sys_r.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), kappa*kinematic_prim_rhythmic, 0, tr_arr  ); 
    if kappa == 1.0
        lw = 5;
        cl = [0.8500 0.3250 0.0980];
    else
        lw = 2.4;
        cl = 0.3*ones( 1, 3 );
    end

    plot( a2, p_roll_r( 1, : ), p_roll_r( 2, : ), 'linewidth', lw, 'color', cl )

    i = i + 1;
end
scatter( a2, 0, 0,  200, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', [0.8500 0.3250 0.0980], 'linewidth', 2.0 )

set( a2, 'xticklabel', {}, 'yticklabel', {}, 'xlim', [-38, 38], 'ylim', [-51, 25] )
exportgraphics( f2, 'images/fig1b_2.pdf', 'ContentType', 'vector');

%% (Image 1c) Dynamic Movement Primitives, Rotational Invariance Property

f1 = figure( ); a1 = axes( 'parent', f1 );
hold on
axis( a1, 'equal' )

% The total time for the rollout
Tr     = 2*data_d.tau;
Nr     = 10000;
tr_arr = linspace( 0, Tr, Nr+1 ); 

rot_deg = linspace( 0, 2*pi, 7 );
rot_deg = rot_deg( 1:end-1 );
i = 1.0;

rot_func = @( deg ) [ cos( deg ), -sin( deg ); sin( deg ), cos( deg ) ];
spc = 3.0;

for rot = rot_deg
    % Kinematic primitive input
    [ p_roll_d, ~, ~] = trans_sys_d.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), rot_func( rot )*kinematic_prim_discrete, 0, tr_arr  ); 
    if rot == 0.0
        lw = 5;
        cl = [0 0.4470 0.7410];
    else
        lw = 2.4;
        cl = 0.3*ones( 1, 3 );
    end

    offset = rot_func( rot ) * [ spc; 0 ];

    plot( a1, offset( 1 ) + p_roll_d( 1, : ), offset( 2 ) + p_roll_d( 2, : ), 'linewidth', lw, 'color', cl )
    scatter( a1, offset( 1 ) + p_roll_d( 1, end ), offset( 2 ) + p_roll_d( 2, end ), 300, 'd','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth', 4 )
    scatter( a1, offset( 1 ) + p_roll_d( 1, 1 ), offset( 2 ) + p_roll_d( 2, 1 ), 300, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth', 4 )

    i = i + 1;
end
set( a1, 'xticklabel', {}, 'yticklabel', {}, 'xlim', [ -15, 15 ], 'ylim', [ -15, 15 ] )
exportgraphics( f1, 'images/fig1c_1.pdf', 'ContentType', 'vector');

f2 = figure( ); a2 = axes( 'parent', f2 );
hold on
axis( a2, 'equal' )

% The total time for the rollout
Tr     = 2*(2*pi*data_r.tau);
Nr     = 10000;
tr_arr = linspace( 0, Tr, Nr+1 ); 

for rot = rot_deg
    % Kinematic primitive input
    [ p_roll_r, ~, ~] = trans_sys_r.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), 0.3*rot_func( rot )*kinematic_prim_rhythmic, 0, tr_arr  ); 
    if rot == 0.0
        lw = 5;
        cl = [0.8500 0.3250 0.0980];
    else
        lw = 2.4;
        cl = 0.3*ones( 1, 3 );
    end

    offset = rot_func( rot ) * [ 7; 0 ];

    plot( a2, offset( 1 ) + p_roll_r( 1, : ), offset( 2 ) + p_roll_r( 2, : ), 'linewidth', lw, 'color', cl )
    scatter( a2, offset( 1 ) + p_roll_r( 1, 1 ), offset( 2 ) + p_roll_r( 2, 1 ), 300, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth', 4 )

    i = i + 1;
end

set( a2, 'xticklabel', {}, 'yticklabel', {}, 'xlim', [-13, 13], 'ylim', [-13, 13] )
exportgraphics( f2, 'images/fig1c_2.pdf', 'ContentType', 'vector');


%% (Image 1d) Dynamic Movement Primitives, Temporal Invariance Property

f1 = figure( ); a1 = axes( 'parent', f1 );
hold on
axis( a1, 'equal' )

% The total time for the rollout
Tr     = 2*data_d.tau;
Nr     = 10000;
tr_arr = linspace( 0, Tr, Nr+1 ); 

% time gain array
kappa_t_arr = [ 2.0, 1.0, 0.5 ];
N_data = length( kappa_t_arr );

% trajectory_data 
traj_data = cell( 1, N_data );

i = 1.0;

for kappa_t = kappa_t_arr
    % Kinematic primitive input
    kinematic_prim_discrete = nft_d.calc_forcing_term( kappa_t*tr_arr( 1:end-1 ), data_d.weight, 0, eye( 2 ) ) + data_d.alpha_z*data_d.beta_z*gd;

    [ p_roll_d, ~, ~] = trans_sys_d.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), kinematic_prim_discrete, 0, tr_arr  ); 
    if kappa_t == 1.0
        lw = 5;
        cl = [0 0.4470 0.7410];
    else
        lw = 2.4;
        cl = 0.3*ones( 1, 3 );
    end

    traj_data{ i } = p_roll_d;
    i = i + 1;
end

ratio_arr = (1:8)/8;
N_ratio = length( ratio_arr );

% Plot through the generated trajectory
for i = 1 : N_data
    p_data = traj_data{ i };
    
    if kappa_t_arr( i ) == 1
        lw = 6.0; cl = [0 0.4470 0.7410];
    else
        lw = 3.0; cl = 0.3*ones( 1, 3 );
    end

    for j = 1 : N_ratio
        idx_end = round( Nr*ratio_arr( j ) );
        plot( a1, (j-1)*10 + p_data( 1, 1:idx_end ), (i-1)*15 +p_data( 2, 1:idx_end ), 'linewidth', lw, 'color', cl )
        scatter( a1, (j-1)*10 + p_data( 1, 1 ), (i-1)*15 + p_data( 2, 1 ), 200, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth', 3 )
    end
end

set( a1, 'xticklabel', {}, 'yticklabel', {}, 'xlim', [ -5, 80 ], 'ylim', [ -5.5, 46] )
exportgraphics( f1, 'images/fig1d_1.pdf', 'ContentType', 'vector');

f2 = figure( ); a2 = axes( 'parent', f2 );
hold on
axis( a2, 'equal' )

% The total time for the rollout
Tr     = 2*(2*pi*data_r.tau);
Nr     = 10000;
tr_arr = linspace( 0, Tr, Nr+1 ); 

% trajectory_data 
traj_data = cell( 1, N_data );

i = 1.0;

for kappa_t = kappa_t_arr
    % Kinematic primitive input
    kinematic_prim_rhythmic = nft_r.calc_forcing_term( kappa_t*tr_arr( 1:end-1 ), data_r.weight, 0, eye( 2 ) ) + data_r.alpha_z*data_r.beta_z*gr;

    [ p_roll_r, ~, ~] = trans_sys_r.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), 0.3*kinematic_prim_rhythmic, 0, tr_arr  ); 
    if kappa_t == 1.0
        lw = 5;
        cl = [0.8500 0.3250 0.0980];
    else
        lw = 2.4;
        cl = 0.3*ones( 1, 3 );
    end

    traj_data{ i } = p_roll_r;
    i = i + 1;
end

ratio_arr = (1:8)/8;
N_ratio = length( ratio_arr );

% Plot through the generated trajectory
for i = 1 : N_data
    p_data = traj_data{ i };
    
    if kappa_t_arr( i ) == 1
        lw = 6.0; cl = [0.8500 0.3250 0.0980];
    else
        lw = 3.0; cl = 0.3*ones( 1, 3 );
    end

    for j = 1 : N_ratio
        idx_end = round( Nr*ratio_arr( j ) );
        plot( a2, (j-1)*10 + p_data( 1, 1:idx_end ), (i-1)*15 +p_data( 2, 1:idx_end ), 'linewidth', lw, 'color', cl )
        scatter( a2, (j-1)*10 + p_data( 1, 1 ), (i-1)*15 + p_data( 2, 1 ), 200, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth', 3 )
    end
end

set( a2, 'xticklabel', {}, 'yticklabel', {}, 'xlim', [ -8.0, 80 ], 'ylim', [ -12, 37] )
exportgraphics( f2, 'images/fig1d_2.pdf', 'ContentType', 'vector');

%% (Image 2a) Parallel Combination of Movements, Discrete + Discrete

data_d1 = load( 'A.mat' ); data_d1 = data_d1.data;
data_d2 = load( 'B.mat' ); data_d2 = data_d2.data;

% The number of basis functions, parameter are all identical
N = size( data_d1.weight, 2 );

% The weights and goal locations are only different
W1 = data_d1.weight; W2 = data_d2.weight;
g1 = data_d1.goal;   g2 = data_d2.goal;

% Other parameters are identical, hence calling from a single data set
tau = data_d1.tau;
as  = data_d1.alpha_s;
az  = data_d1.alpha_z;
bz  = data_d1.beta_z;

% Defining the DMPs
cs_d        = CanonicalSystem( 'discrete', tau, as );
trans_sys_d = TransformationSystem( az, bz, cs_d );
nft_d       = NonlinearForcingTerm( cs_d, N );

% The total time for the rollout
Tr     = 2 * tau * (2*pi);
Nr     = 10000;
tr_arr = linspace( 0, Tr, Nr+1 ); 

% Kinematic primitive inputs
prim_discrete1 = nft_d.calc_forcing_term( tr_arr( 1:end-1 ), W1, 0, eye( 2 ), 'trimmed' ) + az * bz * g1;
prim_discrete2 = nft_d.calc_forcing_term( tr_arr( 1:end-1 ), W2, 0, eye( 2 ), 'trimmed' ) + az * bz * g2;

% Combine these two primitives with different gains
kappa_t = 0:0.1:1.0;

f = figure( ); a = axes( 'parent', f );
hold on
i = 1.0;
for gain = kappa_t
    %     cl = [0.0, 0.4470, 0.7410];
    cl = [0.8500 0.3250 0.0980];
    [ p_traj, ~, ~] = trans_sys_d.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), (1-gain)* prim_discrete1 + gain*prim_discrete2, 0, tr_arr  ); 
    plot( a, (i-1)*10+p_traj( 1, : ), p_traj( 2, : ), 'linewidth', 5, 'color', cl)
    scatter( a, (i-1)*10+p_traj( 1, end ), p_traj( 2, end ), 200, 'd','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth', 2 )
    scatter( a, (i-1)*10+p_traj( 1,   1 ), p_traj( 2, 1 ), 200, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth', 2 )

    i = i + 1;
end
axis equal
set( a, 'xticklabel', {}, 'yticklabel', {}, 'xlim', [ -4.0, 110 ], 'ylim', [ -3, 13] )

exportgraphics( f, 'images/fig2a.pdf', 'ContentType', 'vector');

%% (Image 2b) Parallel Combination of Movements, Discrete + Rhythmic

data_d = load( 'A.mat'     ); data_d = data_d.data;
data_r = load( 'heart.mat' ); data_r = data_r.data;

% The number of basis functions, parameter are all identical
N = size( data_d.weight, 2 );

% The weights and goal locations are only different
W1 = data_d.weight; W2 = data_r.weight;
g1 = data_d.goal;   g2 = data_r.goal;

% Other parameters are identical, hence calling from a single data set
tau = data_d1.tau;
as  = data_d1.alpha_s;
az  = data_d1.alpha_z;
bz  = data_d1.beta_z;

% Defining the DMP, discrete
cs_d        = CanonicalSystem( 'discrete', tau, as );
trans_sys_d = TransformationSystem( az, bz, cs_d );
nft_d       = NonlinearForcingTerm( cs_d, N );

% Defining the DMP, rhythmic
cs_r        = CanonicalSystem( 'rhythmic', tau, as );
trans_sys_r = TransformationSystem( az, bz, cs_d );
nft_r       = NonlinearForcingTerm( cs_r, N );

% The total time for the rollout
Tr     = tau*(2*pi);
Nr     = 10000;
tr_arr = linspace( 0, Tr, Nr+1 ); 

% Kinematic primitive inputs
prim_discrete = nft_d.calc_forcing_term( 0.5*tr_arr( 1:end-1 ), W1, 0, eye( 2 ), 'trimmed' ) + az * bz * g1;
prim_rhythmic = nft_r.calc_forcing_term( 10*tr_arr( 1:end-1 ), W2, 0, eye( 2 ) ) + az * bz * g2;

% Combine these two primitives with different gains
kappa_t = 0:0.1:1.0;

f = figure( ); a = axes( 'parent', f );
hold on
i = 1.0;

% Default scaling
scl1 = 0.8;
scl2 = 0.3;

for gain = kappa_t
    cl2 = [0.0000, 0.4470, 0.7410];
    cl1 = [0.8500, 0.3250, 0.0980];
    [ p_traj, ~, ~] = trans_sys_r.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), (1-gain)*scl1*prim_discrete + gain*scl2*prim_rhythmic, 0, tr_arr  ); 
    plot( a, 2+(i-1)*10.0+p_traj( 1, : ), (i-1)*0.7+p_traj( 2, : ), 'linewidth', 5, 'color', (1-gain)*cl1+gain*cl2)
    scatter( a, 2+(i-1)*10.0+p_traj( 1,   1 ), (i-1)*0.7+p_traj( 2, 1 ), 200, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', (1-gain)*cl1+gain*cl2, 'linewidth', 3 )
    i = i + 1;
end

axis equal
set( a, 'xticklabel', {}, 'yticklabel', {}, 'xlim', [ -4.0, 110 ], 'ylim', [ -3, 13] )
exportgraphics( f, 'images/fig2b.pdf', 'ContentType', 'vector');

%% (Image 2c) Parallel Combination of Movements, Rhythmic + Rhythmic

data_r1 = load(  'heart.mat' ); data_r1 = data_r1.data;
data_r2 = load( 'circle.mat' ); data_r2 = data_r2.data;

% The number of basis functions, parameter are all identical
N = size( data_r1.weight, 2 );

% The weights and goal locations are only different
W1 = data_r1.weight; W2 = data_r2.weight;
g1 = data_r1.goal;   g2 = data_r2.goal;

% Other parameters are identical, hence calling from a single data set
tau = data_r1.tau;
az  = data_r1.alpha_z;
bz  = data_r1.beta_z;

% Defining the DMPs
cs_r        = CanonicalSystem( 'rhythmic', tau, 1.0 );
trans_sys_r = TransformationSystem( az, bz, cs_r );
nft_r       = NonlinearForcingTerm( cs_r, N );

% The total time for the rollout
Tr     = 2 * tau * (2*pi);
Nr     = 10000;
tr_arr = linspace( 0, Tr, Nr+1 ); 

% Kinematic primitive inputs
prim_rhythmic1 = nft_r.calc_forcing_term( tr_arr( 1:end-1 ), W1, 0, eye( 2 ) ) + az * bz * g1;
prim_rhythmic2 = nft_r.calc_forcing_term( tr_arr( 1:end-1 ), W2, 0, eye( 2 ) ) + az * bz * g2;

% Combine these two primitives with different gains
kappa_t = 0:0.1:1.0;

f = figure( ); a = axes( 'parent', f );
hold on
i = 1.0;

% Default scaling
scl1 = 0.25;
scl2 = 6.0;
for gain = kappa_t
    %     cl = [0.8500 0.3250 0.0980];
    cl =  [0.0000, 0.4470, 0.7410];
    [ p_traj, ~, ~] = trans_sys_r.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), (1-gain)*scl1*prim_rhythmic1 + gain*scl2*prim_rhythmic2, 0, tr_arr  ); 
    plot( a, 2+(i-1)*10.5+p_traj( 1, : ), 7-(i-1)*0.1+p_traj( 2, : ), 'linewidth', 5, 'color', cl)
    scatter( a, 2+(i-1)*10.5+p_traj( 1, 1 ), 7-(i-1)*0.1+p_traj( 2, 1 ), 200, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth', 2 )
    i = i + 1;
end

axis equal
set( a, 'xticklabel', {}, 'yticklabel', {}, 'xlim', [ -4.0, 110 ], 'ylim', [ -3, 13] )
exportgraphics( f, 'images/fig2c.pdf', 'ContentType', 'vector');

%% (Image 3a) Sequential Combination of Movements, D-D-D

% Sequence Three Discrete Movements
data_d1 = load( 'R.mat' ); data_d1 = data_d1.data;
data_d2 = load( 'A.mat' ); data_d2 = data_d2.data;
data_d3 = load( 'L.mat' ); data_d3 = data_d3.data;

% The number of basis functions, parameter are all identical
N = size( data_d1.weight, 2 );

% The weights and goal locations are only different
W1 = data_d1.weight; W2 = data_d2.weight; W3 = data_d3.weight;
g1 = data_d1.goal;   g2 = data_d2.goal;   g3 = data_d3.goal;

% Other parameters are identical, hence calling from a single data set
tau = data_d1.tau;
as  = data_d1.alpha_s;
az  = data_d1.alpha_z;
bz  = data_d1.beta_z;

% Defining the DMPs
cs_d        = CanonicalSystem( 'discrete', tau, as );
trans_sys_d = TransformationSystem( az, bz, cs_d );
nft_d       = NonlinearForcingTerm( cs_d, N );

% The total time for the rollout
Tr     = 10*tau;
Nr     = 10000;
tr_arr = linspace( 0, Tr, Nr+1 ); 

% Offsets for the second and third movements
offset2 = [ 9;  0 ];
offset3 = [ 19; 12 ];

% Time offset for the movements, index
idx_offset2 = 970;
idx_offset3 = 1940;

% Kinematic primitive inputs
prim_discrete1 = nft_d.calc_forcing_term( tr_arr( 1:end-1 ), W1, 0, eye( 2 ), 'trimmed' ) + az * bz * g1;
prim_discrete2 = nft_d.calc_forcing_term( tr_arr( 1:end-1 ), W2, 0, eye( 2 ), 'trimmed' ) + az * bz * ( g2 + offset2 );
prim_discrete3 = nft_d.calc_forcing_term( tr_arr( 1:end-1 ), W3, 0, eye( 2 ), 'trimmed' ) + az * bz * ( g3 + offset3 );

% Time shift of the discrete movement, 2nd and 3rd
prim_discrete2_shifted = circshift( prim_discrete2, idx_offset2+10, 2 );
prim_discrete3_shifted = circshift( prim_discrete3, idx_offset3, 2 );

% Define the activation functions
basis_func1 = activation_func( Nr, idx_offset2-10, idx_offset2+20 );
basis_func2 = activation_func( Nr, idx_offset3-10, idx_offset3+10 );
act1_func = 1 - basis_func1;
act2_func = basis_func1  - basis_func2;
act3_func = basis_func2;

% Time shift of second movement
Nshift = round( Nr/3 );

scl1 = 0.92;
scl2 = 1.00;
scl3 = 0.92;

prim_before_sum1 = scl1 * act1_func .* prim_discrete1;
prim_before_sum2 = scl2 * act2_func .* prim_discrete2_shifted;
prim_before_sum3 = scl3 * act3_func .* prim_discrete3_shifted;

prim_total = prim_before_sum1 + prim_before_sum2 + prim_before_sum3;

cl = [0 0.4470 0.7410];
f = figure( ); a = axes( 'parent', f );
hold on; axis equal;
[ p_traj, ~, ~] = trans_sys_d.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), prim_total , 0, tr_arr  ); 
% Overlap the plots
[ p_traj1, ~, ~] = trans_sys_d.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), scl1 * prim_discrete1, 0, tr_arr  ); 
[ p_traj2, ~, ~] = trans_sys_d.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), scl2 * prim_discrete2 - scl2 * az * bz * offset2, 0, tr_arr  ); 
[ p_traj3, ~, ~] = trans_sys_d.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), scl3 * prim_discrete3 - scl3 * az * bz * offset3, 0, tr_arr  ); 
plot( a, p_traj1( 1, : ), p_traj1( 2, : ), 'linewidth', 10, 'color', cl, 'linestyle', ':' )
plot( a, p_traj2( 1, : ) + scl2*offset2( 1 ), p_traj2( 2, : ) + scl2*offset2( 2 ), 'linewidth', 10, 'color', cl, 'linestyle', ':' )
plot( a, p_traj3( 1, : ) + scl3*offset3( 1 ), p_traj3( 2, : ) + scl3*offset3( 2 ), 'linewidth', 10, 'color', cl, 'linestyle', ':' )

scatter( a, p_traj1( 1, 1 ), p_traj1( 2, 1 ), 400, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth', 8 )
scatter( a, p_traj2( 1, 1 ) + scl2*offset2( 1 ), p_traj2( 2, 1 ) + scl2*offset2( 2 ), 400, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth',8 )
scatter( a, p_traj3( 1, 1 ) + scl3*offset3( 1 ), p_traj3( 2, 1 ) + scl3*offset3( 2 ), 400, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth',8 )

scatter( a, p_traj1( 1, end ), p_traj1( 2, end ), 400, 'd','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth', 6 )
scatter( a, p_traj2( 1, end ) + scl2*offset2( 1 ), p_traj2( 2, end ) + scl2*offset2( 2 ), 400, 'd','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth',6 )
scatter( a, p_traj3( 1, end ) + scl3*offset3( 1 ), p_traj3( 2, end ) + scl3*offset3( 2 ), 400, 'd','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth',6 )

plot( a, p_traj( 1, : ), p_traj( 2, : ), 'color', 'k', 'linewidth', 3 )

set( a, 'xticklabel', {}, 'yticklabel', {}, 'xlim', [-7, 30], 'ylim', [-5, 15])

exportgraphics( f, 'images/fig3a.pdf', 'ContentType', 'vector');

%% (Image 3b) Sequential Combination of Movements, D-R-D

% Sequence Three Discrete Movements
data_d1 = load( 'R.mat' );      data_d1 = data_d1.data;
data_r1 = load( 'circle.mat' ); data_r1 = data_r1.data;
data_d2 = load( 'B.mat' );      data_d2 = data_d2.data;

% The number of basis functions, parameter are all identical
N = size( data_d1.weight, 2 );

% The weights and goal locations are only different
W1 = data_d1.weight; W2 = data_r1.weight; W3 = data_d2.weight;
g1 = data_d1.goal;   g2 = data_r1.goal;   g3 = data_d2.goal;

% Other parameters are identical, hence calling from a single data set
tau = data_d1.tau;
as  = data_d1.alpha_s;
az  = data_d1.alpha_z;
bz  = data_d1.beta_z;

% Defining the DMPs, Discrete
cs_d        = CanonicalSystem( 'discrete', tau, as );
trans_sys_d = TransformationSystem( az, bz, cs_d );
nft_d       = NonlinearForcingTerm( cs_d, N );

% Defining the DMPs, Rhythmic
cs_r        = CanonicalSystem( 'rhythmic', tau, 1.0 );
trans_sys_r = TransformationSystem( az, bz, cs_r );
nft_r       = NonlinearForcingTerm( cs_r, N );

% The total time for the rollout
Tr     = 10*tau;
Nr     = 10000;
tr_arr = linspace( 0, Tr, Nr+1 ); 

% Offsets for the second and third movements
offset2 = [ 2; 0.5 ];
offset3 = [ 17.5; 0.5 ];

% Time offset for the movements, index
idx_rhythmic_off = 500;
idx_offset2 = 970;
idx_offset3 = 4500;

% Kinematic primitive inputs
prim_discrete1 = nft_d.calc_forcing_term( tr_arr( 1:end-1 ), W1, 0, eye( 2 ), 'trimmed' ) + az * bz * g1;
prim_rhythmic1 = nft_r.calc_forcing_term( 6*tr_arr( 1:end-1 ), W2, 0, eye( 2 ), 'trimmed' ) + az * bz * ( g2 + offset2 );
prim_discrete2 = nft_d.calc_forcing_term( tr_arr( 1:end-1 ), W3, 0, eye( 2 ), 'trimmed' ) + az * bz * ( g3 + offset3 );

% Time shift of the discrete movement, 2nd and 3rd
prim_rhythmic1_shifted = circshift( prim_rhythmic1, idx_rhythmic_off , 2 );
prim_discrete2_shifted = circshift( prim_discrete2, idx_offset3+30, 2 );

% Define the activation functions
basis_func1 = activation_func( Nr, idx_offset2, idx_offset2+3 );
basis_func2 = activation_func( Nr, idx_offset3, idx_offset3+2 );
act1_func = 1 - basis_func1;
act2_func = basis_func1  - basis_func2;
act3_func = basis_func2;

% Time shift of second movement
Nshift = round( Nr/3 );

scl1 = 0.92;
scl2 = 7.00;
scl3 = 0.92;

prim_before_sum1 = scl1 * act1_func .* prim_discrete1;
prim_before_sum2 = scl2 * act2_func .* prim_rhythmic1_shifted;
prim_before_sum3 = scl3 * act3_func .* prim_discrete2_shifted;

prim_total = prim_before_sum1 + prim_before_sum2 + prim_before_sum3;

f = figure( ); a = axes( 'parent', f );
hold on; axis equal;
[ p_traj, ~, ~] = trans_sys_d.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), prim_total , 0, tr_arr); 

% Overlap the plots
[ p_traj1, ~, ~] = trans_sys_d.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), scl1 * prim_discrete1, 0, tr_arr  ); 
[ p_traj2, ~, ~] = trans_sys_r.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), scl2 * prim_rhythmic1 - scl2 * az * bz * offset2, 0, tr_arr  ); 
[ p_traj3, ~, ~] = trans_sys_d.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), scl3 * prim_discrete2 - scl3 * az * bz * offset3, 0, tr_arr  ); 

cld = [0.0000 0.4470 0.7410];
clo = [0.8500 0.3250 0.0980];

plot( a, p_traj1( 1, : ), p_traj1( 2, : ), 'linewidth', 10, 'color', cld, 'linestyle', ':' )
plot( a, p_traj2( 1, 1:1080 ) + scl2*offset2( 1 ), p_traj2( 2, 1:1080  ) + scl2*offset2( 2 ), 'linewidth', 10, 'color', clo, 'linestyle', ':' )
plot( a, p_traj3( 1, : ) + scl3*offset3( 1 ), p_traj3( 2, : ) + scl3*offset3( 2 ), 'linewidth', 10, 'color', cld, 'linestyle', ':' )

scatter( a, p_traj1( 1, 1 ), p_traj1( 2, 1 ), 400, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth', 8 )
scatter( a, p_traj3( 1, 1 ) + scl3*offset3( 1 ), p_traj3( 2, 1 ) + scl3*offset3( 2 ), 400, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth',8 )

scatter( a, p_traj1( 1, end ), p_traj1( 2, end ), 400, 'd','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth', 6 )
scatter( a, p_traj3( 1, end ) + scl3*offset3( 1 ), p_traj3( 2, end ) + scl3*offset3( 2 ), 400, 'd','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth',6 )

plot( a, p_traj( 1, : ), p_traj( 2, : ), 'color', 'k', 'linewidth', 3 )

set( a, 'xticklabel', {}, 'yticklabel', {}, 'xlim', [-6, 29], 'ylim', [-5, 15])

exportgraphics( f, 'images/fig3b.pdf', 'ContentType', 'vector');


%% (Image 3c) Sequential Combination of Movements, D-R-D, Obstacle Avoidance

% Sequence Three Discrete Movements
data_d1 = load( 'R.mat' );      data_d1 = data_d1.data;
data_r1 = load( 'circle.mat' ); data_r1 = data_r1.data;
data_d2 = load( 'B.mat' );      data_d2 = data_d2.data;

% Color
cl = [0.0000 0.4470 0.7410];

% The number of basis functions, parameter are all identical
N = size( data_d1.weight, 2 );

% The weights and goal locations are only different
W1 = data_d1.weight; W2 = data_r1.weight; W3 = data_d2.weight;
g1 = data_d1.goal;   g2 = data_r1.goal;   g3 = data_d2.goal;

% Other parameters are identical, hence calling from a single data set
tau = data_d1.tau;
as  = data_d1.alpha_s;
az  = data_d1.alpha_z;
bz  = data_d1.beta_z;

% Defining the DMPs, Discrete
cs_d        = CanonicalSystem( 'discrete', tau, as );
trans_sys_d = TransformationSystem( az, bz, cs_d );
nft_d       = NonlinearForcingTerm( cs_d, N );

% Defining the DMPs, Rhythmic
cs_r        = CanonicalSystem( 'rhythmic', tau, 1.0 );
trans_sys_r = TransformationSystem( az, bz, cs_r );
nft_r       = NonlinearForcingTerm( cs_r, N );

% The total time for the rollout
Tr     = 10*tau;
Nr     = 10000;
tr_arr = linspace( 0, Tr, Nr+1 ); 

% Offsets for the second and third movements
offset2 = [ 2; 0.5 ];
offset3 = [ 17.5; 0.5 ];

% Time offset for the movements, index
idx_rhythmic_off = 500;
idx_offset2 = 970;
idx_offset3 = 4500;

% Kinematic primitive inputs
prim_discrete1 = nft_d.calc_forcing_term( tr_arr( 1:end-1 ), W1, 0, eye( 2 ), 'trimmed' ) + az * bz * g1;
prim_rhythmic1 = nft_r.calc_forcing_term( 6*tr_arr( 1:end-1 ), W2, 0, eye( 2 ), 'trimmed' ) + az * bz * ( g2 + offset2 );
prim_discrete2 = nft_d.calc_forcing_term( tr_arr( 1:end-1 ), W3, 0, eye( 2 ), 'trimmed' ) + az * bz * ( g3 + offset3 );

% Time shift of the discrete movement, 2nd and 3rd
prim_rhythmic1_shifted = circshift( prim_rhythmic1, idx_rhythmic_off , 2 );
prim_discrete2_shifted = circshift( prim_discrete2, idx_offset3+30, 2 );

% Define the activation functions
basis_func1 = activation_func( Nr, idx_offset2, idx_offset2+3 );
basis_func2 = activation_func( Nr, idx_offset3, idx_offset3+2 );
act1_func = 1 - basis_func1;
act2_func = basis_func1  - basis_func2;
act3_func = basis_func2;

% Time shift of second movement
Nshift = round( Nr/3 );

scl1 = 0.92;
scl2 = 7.00;
scl3 = 0.92;

prim_before_sum1 = scl1 * act1_func .* prim_discrete1;
prim_before_sum2 = scl2 * act2_func .* prim_rhythmic1_shifted;
prim_before_sum3 = scl3 * act3_func .* prim_discrete2_shifted;

prim_total = prim_before_sum1 + prim_before_sum2 + prim_before_sum3;

% Add obstacle avoidance 
f = figure( ); a = axes( 'parent', f );
hold on; axis equal;
[ p_traj, ~, ~] = trans_sys_d.rollout_w_obs_avoid( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), prim_total , 0, tr_arr, [22,8] ); 

% Overlap the plots
[ p_traj1, ~, ~] = trans_sys_d.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), scl1 * prim_discrete1, 0, tr_arr  ); 
[ p_traj2, ~, ~] = trans_sys_r.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), scl2 * prim_rhythmic1 - scl2 * az * bz * offset2, 0, tr_arr  ); 
[ p_traj3, ~, ~] = trans_sys_d.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), scl3 * prim_discrete2 - scl3 * az * bz * offset3, 0, tr_arr  ); 

cld = [0.0000 0.4470 0.7410];
clo = [0.8500 0.3250 0.0980];

plot( a, p_traj1( 1, : ), p_traj1( 2, : ), 'linewidth', 10, 'color', cld, 'linestyle', ':' )
plot( a, p_traj2( 1, 1:1080 ) + scl2*offset2( 1 ), p_traj2( 2, 1:1080  ) + scl2*offset2( 2 ), 'linewidth', 10, 'color', clo, 'linestyle', ':' )
plot( a, p_traj3( 1, : ) + scl3*offset3( 1 ), p_traj3( 2, : ) + scl3*offset3( 2 ), 'linewidth', 10, 'color', cld, 'linestyle', ':' )

scatter( a, p_traj1( 1, 1 ), p_traj1( 2, 1 ), 400, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth', 8 )
scatter( a, p_traj3( 1, 1 ) + scl3*offset3( 1 ), p_traj3( 2, 1 ) + scl3*offset3( 2 ), 400, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth',8 )

scatter( a, p_traj1( 1, end ), p_traj1( 2, end ), 400, 'd','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth', 6 )
scatter( a, p_traj3( 1, end ) + scl3*offset3( 1 ), p_traj3( 2, end ) + scl3*offset3( 2 ), 400, 'd','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth',6 )

plot( a, p_traj( 1, : ), p_traj( 2, : ), 'color', 'k', 'linewidth', 3 )

set( a, 'xticklabel', {}, 'yticklabel', {}, 'xlim', [-6, 29], 'ylim', [-5, 15])

exportgraphics( f, 'images/fig3b.pdf', 'ContentType', 'vector');

%% (Image 4) Parallel and Sequential Combinations of Movements

% Sequence Three Discrete Movements
data_d1 = load( 'R.mat' );      data_d1 = data_d1.data;
data_r1 = load( 'circle.mat' ); data_r1 = data_r1.data;
data_d2 = load( 'B.mat' );      data_d2 = data_d2.data;

data_r2 = load( 'heart.mat' );  data_r2 = data_r2.data;
data_r3 = load( 'eight.mat' );  data_r3 = data_r3.data;

% The number of basis functions, parameter are all identical
N = size( data_d1.weight, 2 );

% The weights and goal locations are only different
W1 = data_d1.weight; W2 = data_r1.weight; W3 = data_d2.weight;
g1 = data_d1.goal;   g2 = data_r1.goal;   g3 = data_d2.goal;

% Added movements
W4 = data_r2.weight; W5 = data_r3.weight;
g4 = data_r2.goal;   g5 = data_r3.goal;

% Other parameters are identical, hence calling from a single data set
tau = data_d1.tau;
as  = data_d1.alpha_s;
az  = data_d1.alpha_z;
bz  = data_d1.beta_z;

% Defining the DMPs, Discrete
cs_d        = CanonicalSystem( 'discrete', tau, as );
trans_sys_d = TransformationSystem( az, bz, cs_d );
nft_d       = NonlinearForcingTerm( cs_d, N );

% Defining the DMPs, Rhythmic
cs_r        = CanonicalSystem( 'rhythmic', tau, 1.0 );
trans_sys_r = TransformationSystem( az, bz, cs_r );
nft_r       = NonlinearForcingTerm( cs_r, N );

% The total time for the rollout
Tr     = 6*tau*(2*pi);
Nr     = 30000;
tr_arr = linspace( 0, Tr, Nr+1 ); 

% Offsets for the second and third movements
offset2 = [ 2.1; 0.5 ];
offset3 = [ 19.5; 0.5 ];

% Time offset for the movements, index
idx_rhythmic_off = -2200;
idx_offset2 = 780;
idx_offset3 = 7500;

% Kinematic primitive inputs
prim_discrete1 = nft_d.calc_forcing_term( tr_arr( 1:end-1 ), W1, 0, eye( 2 ), 'trimmed' ) + az * bz * g1;
prim_rhythmic1 = nft_r.calc_forcing_term( tr_arr( 1:end-1 ), W2, 0, eye( 2 ), 'trimmed' ) + az * bz * ( g2 + offset2 );
prim_discrete2 = nft_d.calc_forcing_term( 0.2*tr_arr( 1:end-1 ), W3, 0, eye( 2 ), 'trimmed' ) + az * bz * ( g3 + offset3 );

prim_rhythmic2 = nft_r.calc_forcing_term( 12*tr_arr( 1:end-1 ), W4, 0, eye( 2 ), 'trimmed' );
prim_rhythmic3 = nft_r.calc_forcing_term( 24*tr_arr( 1:end-1 ), W5, 0, eye( 2 ), 'trimmed' );

% Time shift of the discrete movement, 2nd and 3rd
prim_rhythmic1_shifted = circshift( prim_rhythmic1, idx_rhythmic_off , 2 );
prim_rhythmic2_shifted = circshift( prim_rhythmic2, idx_rhythmic_off , 2 );

prim_discrete2_shifted = circshift( prim_discrete2, idx_offset3+30, 2 );

% Define the activation functions
basis_func1 = activation_func( Nr, idx_offset2-20, idx_offset2+30 );
basis_func2 = activation_func( Nr, idx_offset3, idx_offset3+2 );
act1_func = 1 - basis_func1;
act2_func = basis_func1  - basis_func2;
act3_func = basis_func2;

% Time shift of second movement
Nshift = round( Nr/3 );

scl1 = 0.92;
scl2 = 7.00;
scl3 = 0.92;

prim_before_sum1 = scl1 * act1_func .* prim_discrete1;
prim_before_sum2 = scl2 * act2_func .* ( prim_rhythmic1_shifted + 0.01*prim_rhythmic2_shifted );
prim_before_sum3 = scl3 * act3_func .* ( prim_discrete2_shifted + 0.6*prim_rhythmic3 );

prim_total = prim_before_sum1 + prim_before_sum2 + prim_before_sum3;

f = figure( ); a = axes( 'parent', f );
hold on; axis equal;
[ p_traj, ~, ~] = trans_sys_d.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), prim_total , 0, tr_arr); 

% Overlap the plots
[ p_traj1, ~, ~] = trans_sys_d.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), scl1 * prim_discrete1, 0, tr_arr  ); 
[ p_traj2, ~, ~] = trans_sys_r.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), scl2 * prim_rhythmic1 - scl2 * az * bz * offset2, 0, tr_arr  ); 
[ p_traj3, ~, ~] = trans_sys_d.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), scl3 * prim_discrete2 - scl3 * az * bz * offset3, 0, tr_arr  ); 

cld = [0.0000 0.4470 0.7410];
clo = [0.8500 0.3250 0.0980];

plot( a, p_traj1( 1, : ), p_traj1( 2, : ), 'linewidth', 10, 'color', cld, 'linestyle', ':' )
plot( a, p_traj2( 1, 1:8080 ) + scl2*offset2( 1 ), p_traj2( 2, 1:8080  ) + scl2*offset2( 2 ), 'linewidth', 10, 'color', clo, 'linestyle', ':' )
plot( a, p_traj3( 1, : ) + scl3*offset3( 1 ), p_traj3( 2, : ) + scl3*offset3( 2 ), 'linewidth', 10, 'color', cld, 'linestyle', ':' )

scatter( a, p_traj1( 1, 1 ), p_traj1( 2, 1 ), 400, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth', 8 )
scatter( a, p_traj3( 1, 1 ) + scl3*offset3( 1 ), p_traj3( 2, 1 ) + scl3*offset3( 2 ), 400, 'o','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth',8 )

scatter( a, p_traj1( 1, end ), p_traj1( 2, end ), 400, 'd','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth', 6 )
scatter( a, p_traj3( 1, end ) + scl3*offset3( 1 ), p_traj3( 2, end ) + scl3*offset3( 2 ), 400, 'd','filled', 'markerfacecolor', 'w', 'markeredgecolor', cl, 'linewidth',6 )

plot( a, p_traj( 1, : ), p_traj( 2, : ), 'color', 'k', 'linewidth', 3 )

set( a, 'xticklabel', {}, 'yticklabel', {}, 'xlim', [-6, 29], 'ylim', [-5, 15])

exportgraphics( f, 'images/fig4_parallel_sequential.pdf', 'ContentType', 'vector');
