function data = read_data(filename)
    %READ_DATA  Read trajectory data into a struct
    %
    %   data = read_data(filename) returns a struct with fields
    %     .t         time vector
    %     .p         3D positions
    %     .R         orientation matrices
    %     .axis      rotation axes
    %     .angle     rotation angles
    %     .exp_coord exponential coordinates (axis.*angle)

    % Read the time data 
    data.t = read_column(filename, 'time')';

    % Read the 3D position and orientation data
    data.p = read_position(filename);
    data.R = read_orientation(filename);

    % Read axis‐angle and compute exponential coordinates
    [data.axis, data.angle] = read_axisangle(filename);
    data.e = data.axis .* data.angle;
end
