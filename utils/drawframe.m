function drawframe(ax, position, orientation, arrowlength, arrowwidth, markersize)
% DRAWFRAME Draws a 3D coordinate frame at a given position and orientation
%
%   drawframe(ax, position, orientation, arrowlength, markercolor, markersize)
%
%   Inputs:
%       ax          - axes handle
%       position    - 3x1 or 1x3 vector (origin of the frame)
%       orientation - 3x3 rotation matrix (special orthogonal)
%       arrowlength - scalar length for the axis arrows
%       markercolor - RGB triplet or color char for the marker
%       markersize  - size of the marker

    % Validate axis handle
    if ~ishandle(ax) || ~strcmp(get(ax, 'type'), 'axes')
        error('First argument must be a valid axes handle.');
    end

    % Ensure column vector for position
    position = position(:);

    % Define axis directions
    x_dir = orientation(:,1);
    y_dir = orientation(:,2);
    z_dir = orientation(:,3);

    % Plot axis arrows
    quiver3(ax, position(1), position(2), position(3), ...
            arrowlength * x_dir(1), arrowlength * x_dir(2), arrowlength * x_dir(3), ...
            'Color', [1 0 0], 'LineWidth', arrowwidth, 'MaxHeadSize', 0.5); % X - red

    quiver3(ax, position(1), position(2), position(3), ...
            arrowlength * y_dir(1), arrowlength * y_dir(2), arrowlength * y_dir(3), ...
            'Color', [0 1 0], 'LineWidth', arrowwidth, 'MaxHeadSize', 0.5); % Y - green

    quiver3(ax, position(1), position(2), position(3), ...
            arrowlength * z_dir(1), arrowlength * z_dir(2), arrowlength * z_dir(3), ...
            'Color', [0 0 1], 'LineWidth', arrowwidth, 'MaxHeadSize', 0.5); % Z - blue

    % Plot marker at the origin
    plot3(ax, position(1), position(2), position(3), ...
          'o', ...
          'MarkerSize', markersize, ...
          'MarkerEdgeColor', 'k', ...
          'MarkerFaceColor', 'w');
end
