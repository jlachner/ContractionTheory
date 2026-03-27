function [ raw_data, smth_data ]= diff_smooth( data, time, window )

    % The raw data that is differentiated
    raw_data = gradient( data, time, 2 );

    % The smoothen version of the data
    smth_data = smoothdata( raw_data, 2, 'sgolay', window );

end
