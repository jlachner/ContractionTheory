function col = read_column(filename, colname)
%READ_COLUMN  Read a single named column from a tab-delimited text file.
%   col = READ_COLUMN(filename, colname) returns the numeric data in
%   column colname of the file filename. The file must have a header
%   row with variable names matching colname.

    % 1) Set up import options for a text file with tabs
    opts = detectImportOptions(filename, ...
        'FileType','text', ...
        'Delimiter','\t');
    
    % 2) Tell it to read only the requested column
    if ~ismember(colname, opts.VariableNames)
        error('Column "%s" not found in %s.', colname, filename);
    end
    opts.SelectedVariableNames = {colname};
    
    % 3) Read into a table, then extract
    T = readtable(filename, opts);
    col = T.(colname);
end
