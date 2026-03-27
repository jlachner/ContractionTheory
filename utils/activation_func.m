function output = activation_func(N, nstart, nend)
    % Validate inputs
    assert(nend > nstart, 'nend must be greater than nstart.');
    assert(nstart >= 1 && nend <= N, 'nstart and nend must be within the bounds of the array size.');

    % Initialize the output array with zeros
    output = zeros(1, N);
    
    % Compute values using the sigmoid function
    for i = 1:N
        if i < nstart
            output(i) = 0;
        elseif i > nend
            output(i) = 1;
        else
            t = (i - nstart) / (nend - nstart);
            output(i) = 3 * t^2 - 2 * t^3;
        end
    end
end
