function p_arr = read_position( filename )
%READ_POSITION Read x0 y0 z0 and save it as an array
    x0 = read_column( filename, 'x0' );
    y0 = read_column( filename, 'y0' );
    z0 = read_column( filename, 'z0' );

    p_arr = [ x0, y0, z0 ]';
end
