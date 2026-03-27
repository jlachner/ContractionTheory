%% Initialization 
clear all; close all; clc;

% Change the directory to the current directory 
cd( fileparts(matlab.desktop.editor.getActiveFilename) );

%% (Section 1) Draw Trajectory

% Define the time symbol to derive the analytical form
syms t_sym

% The three types of trajectory that we aim to learn
% [1] circle
% [2] heart
% [3] eight
name_traj = 'heart';

% Period is chosen such that the tau_r results in 1 second
Tp = 2*pi;       

switch name_traj

    case 'circle'
        r = 0.5;
        t = 2*pi/Tp * t_sym;
        x = r * cos( t );
        y = r * sin( t );

    case 'heart'
        t = 2*pi/Tp * t_sym;
        x = 16 * sin( t )^3;
        y = 13 * cos( t ) - 5 * cos( 2*t )- 2 * cos( 3*t ) - cos( 4*t );

    case 'eight'
        t = 2*pi/Tp * t_sym;
        x = 2 * sin( t );
        y = 2 * sin( t ) * cos( t );

    otherwise
        error( 'Wrong input: %s', name_traj );  
end

  p_sym = [ x;y ]; 
 dp_sym = diff(  p_sym, t_sym );
ddp_sym = diff( dp_sym, t_sym );

  p_func = matlabFunction(   p_sym );
 dp_func = matlabFunction(  dp_sym );
ddp_func = matlabFunction( ddp_sym );

t_arr = linspace( 0, Tp, 1200 );

% Since it is a cyclic input, we do not need the final value
  t_arr = t_arr( 1:end-1 );
  p_arr =   p_func( t_arr );
 dp_arr =  dp_func( t_arr );
ddp_arr = ddp_func( t_arr );

f = figure( ); a = axes( 'parent', f );
plot( a, p_arr( 1, : ), p_arr( 2, : ), 'linewidth', 3 )

%% (Section 2) Save Trajectory Data

% Get the length of the t_arr
P = length( t_arr );

% Offset out the initial condition
p_arr = p_arr - p_arr( :, 1 );

% Save the data under raw_data
data = struct( );
data.t_arr    =   t_arr;
data.p_data   =   p_arr;
data.dp_data  =  dp_arr;
data.ddp_data = ddp_arr;

% Save the position, velocity and acceleration data
save( ['raw_data/rhythmic/', name_traj, '.mat'] , 'data' );
