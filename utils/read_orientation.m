function R_arr = read_orientation( filename )
%READ_POSITION Read x0 y0 z0 and save it as an array
    R11 = read_column( filename, 'R0_11' );
    R12 = read_column( filename, 'R0_12' );
    R13 = read_column( filename, 'R0_13' );
    R21 = read_column( filename, 'R0_21' );
    R22 = read_column( filename, 'R0_22' );
    R23 = read_column( filename, 'R0_23' );
    R31 = read_column( filename, 'R0_31' );
    R32 = read_column( filename, 'R0_32' );
    R33 = read_column( filename, 'R0_33' );
   
    R_arr = zeros( 3, 3, length( R11 ) );

    R_arr( 1, 1, : ) = R11;
    R_arr( 1, 2, : ) = R12;
    R_arr( 1, 3, : ) = R13;
    R_arr( 2, 1, : ) = R21;
    R_arr( 2, 2, : ) = R22;
    R_arr( 2, 3, : ) = R23;
    R_arr( 3, 1, : ) = R31;
    R_arr( 3, 2, : ) = R32;
    R_arr( 3, 3, : ) = R33;
    
    % [2025-05-25]
    % Code error, R flipped

end
