function y = rightshift_with_wrap(x, Nt)
% RIGHTSHIFT_WITH_WRAP Right-shifts each row of x by Nt steps.
%   The left is filled with the row's first element,
%   and the result is trimmed to original size.
%
%   x  : MxN matrix or vector
%   Nt : number of steps to shift right
%
%   y  : same size as x

    [M, N] = size(x);

    if isvector(x)
        % Preserve vector shape
        if isrow(x)
            filler = repmat(x(1), 1, Nt);
            y = [filler, x];
            y = y(1:N);
        else
            filler = repmat(x(1), Nt, 1);
            y = [filler; x];
            y = y(1:N);
        end
    else
        % Matrix case: shift each row
        y = zeros(M, N);
        for i = 1:M
            row = x(i, :);
            fillval = row(1);
            filler = repmat(fillval, 1, Nt);
            shifted = [filler, row];
            y(i, :) = shifted(1:N);
        end
    end
end
