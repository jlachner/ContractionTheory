%% Initialization 
clear all; close all; clc;

% Change the directory to the current directory 
cd( fileparts( matlab.desktop.editor.getActiveFilename ) );

% Add the directory, "Bezier Toolbox" from Robbin van Hoek (RvH)
% Please refer to the "license.txt" under "RvH_Bezier_Toolbox"
% Code under "RvH_Bezier_Toolbox" has a Copyright (c) 2018, Robbin van Hoek
% Source code link: https://www.mathworks.com/matlabcentral/fileexchange/69302-bezier-toolbox
addpath( "RvH_Bezier_Toolbox" )

%% Draw the trajectory and save the Bezier Parameters

% Calling the interactive Bezier curve tool box
Runit_interactiveBezier

%% Check the parameters and plot the trajectory

f = figure( ); a = axes( 'parent', f );
axis equal
hold( a, 'on' )

% Parameters saved as Bezier structure
% The number of trajectories 
Ntraj = length( Bez );
Nd    = size( Bez( 1 ).Q, 2 );
Ns    = 200;

% Whole x y data and its time array 
x_arr = zeros( 1, Ns*Ntraj );
y_arr = zeros( 1, Ns*Ntraj );
t_arr = zeros( 1, Ns*Ntraj );
t     = linspace( 0, 1, Ns );

% Iterate through each Bezier curve
for i = 1 : Ntraj
    [ C, theta, K, L]  = BezierEval( Bez( i ).Q, t, Bez( i ).w ); %rational
    plot( a, C( 1, : ), C( 2, : ) ) 
    x_arr( (i-1)*Ns+1: i*Ns ) = C( 1, : );
    y_arr( (i-1)*Ns+1: i*Ns ) = C( 2, : );
    t_arr( (i-1)*Ns+1: i*Ns ) = t + (i-1);
end

% Normalization
t_arr = t_arr/max( t_arr );

% Once we get the whole x_arr and y_arr, we set the origin as 0 
x_arr = x_arr - x_arr( 1 );
y_arr = y_arr - y_arr( 1 );

% Fitting the function, which was manually discovered
myfittype = fittype("( 6*(t/a)^5-15*(t/a)^4+10*(t/a)^3 ) *( b21*(t-a)^21 + b20*(t-a)^20 + b19*(t-a)^19 + b18*(t-a)^18 + b17*(t-a)^17 + b16*(t-a)^16 + b15*(t-a)^15 + b14*(t-a)^14 + b13*(t-a)^13 + b12*(t-a)^12 + b11*(t-a)^11 + b10*(t-a)^10 + b9*(t-a)^9 + b8*(t-a)^8 + b7*(t-a)^7 + b6 * (t-a)^6 + b5 * (t-a)^5 + b4 * (t-a)^4 + b3*(t-a)^3 + c )", ...
                'independent',"t", 'coefficients', [ "b3" "b4" "b5" "b6" "b7" "b8" "b9" "b10" "b11" "b12" ...
                                                     "b13" "b14" "b15" "b16" "b17" "b18" "b19" "b20" "b21" ], 'problem' ,["a", "c"] );
% 
% myfittype = fittype("( 6*(t/a)^5-15*(t/a)^4+10*(t/a)^3 ) *(  + b5 * (t-a)^5 + b4 * (t-a)^4 + b3*(t-a)^3 + c )", ...
%                 'independent',"t", 'coefficients', [ "b3" "b4" "b5"  ], 'problem' ,["a", "c"] );

% Specify the value for a_fix, c_fix
a_fix = max( t_arr );
c_fix = x_arr( end );

% Perform the fit, passing the fixed 'b' as a 'problem' parameter
f = figure( ); a = axes( 'parent', f );
[fitresult_x, ~] = fit( t_arr', x_arr', myfittype, 'problem', { a_fix, c_fix } );
plot( a, t_arr, fitresult_x( t_arr ), t_arr, x_arr )
axis equal

% Specify the value for 'b'
a_fix = max( t_arr );
c_fix = y_arr( end );

% Perform the fit, passing the fixed 'b' as a 'problem' parameter
f = figure( ); a = axes( 'parent', f );
[fitresult_y, ~] = fit( t_arr', y_arr', myfittype, 'problem', { a_fix, c_fix } );
plot( a, t_arr, fitresult_y( t_arr ), t_arr, y_arr )
axis equal

% Double-check with the plot
f = figure( ); a = axes( 'parent', f );
plot( a, fitresult_x( t_arr ), fitresult_y( t_arr ), 'linewidth', 1, 'color', 'k' );
hold on
plot( a, x_arr, y_arr, 'linewidth', 1, 'color', 'c' );
axis equal

% Saving also the derivatives
[ dfx, ddfx ] = differentiate( fitresult_x, t_arr );
[ dfy, ddfy ] = differentiate( fitresult_y, t_arr );

% Saving the data for DMP MATLAB
  p_data = [   fitresult_x( t_arr ),   fitresult_y( t_arr )]';
 dp_data = [           dfx,         dfy   ]';
ddp_data = [          ddfx,        ddfy   ]';

data = struct( );
data.t_arr    =    t_arr;
data.p_data   =   p_data;
data.dp_data  =  dp_data;
data.ddp_data = ddp_data;

% Save the position, velocity and acceleration data
save( './raw_data/discrete/tmp.mat', 'data' );
