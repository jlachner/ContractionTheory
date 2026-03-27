function [ axis, angle ] = read_axisangle( filename )
%READ_POSITION Read x0 y0 z0 and save it as an array
    tmp1 = read_column( filename, 'u_x'   );
    tmp2 = read_column( filename, 'u_y'   );
    tmp3 = read_column( filename, 'u_z'   );
    
    axis = zeros( 3, length( tmp1 ) );
    
    axis( 1, : ) = tmp1;
    axis( 2, : ) = tmp2;
    axis( 3, : ) = tmp3;

    angle = read_column( filename, 'theta' )';
    angle = mod( angle, pi );
    
end
