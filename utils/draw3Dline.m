function draw3Dline(ax, traj, linestyle, color, linewidth, mksize)
% DRAW3DLINE Draws a 3D line from a 3xN trajectory with markers at endpoints
%
%   draw3Dline(ax, traj, linestyle, color, linewidth, mksize)
%
%   Inputs:
%       ax        - axes handle for plotting
%       traj      - 3xN matrix of 3D positions
%       linestyle - line style string (e.g., '-', '--', ':')
%       color     - RGB triplet or color char (e.g., 'r', [0.1 0.5 0.9])
%       linewidth - width of the plotted line
%       mksize    - marker size for start/end points

    % Validate inputs
    if nargin < 6
        error('All six input arguments are required.');
    end
    if ~ishandle(ax) || ~strcmp(get(ax, 'type'), 'axes')
        error('First argument must be a valid axes handle.');
    end
    if size(traj,1) ~= 3
        error('Trajectory must be a 3xN matrix.');
    end

    % Extract start and end positions
    start_pos = traj(:,1);
    end_pos   = traj(:,end);

    % Plot the 3D line
    plot3(ax, traj(1,:), traj(2,:), traj(3,:), ...
          'LineStyle', linestyle, ...
          'Color', color, ...
          'LineWidth', linewidth);

    % Start marker: circle
    plot3(ax, start_pos(1), start_pos(2), start_pos(3), ...
          'o', ...
          'MarkerSize', mksize, ...
          'MarkerEdgeColor', color, ...
          'MarkerFaceColor', 'w');

    % End marker: diamond
    plot3(ax, end_pos(1), end_pos(2), end_pos(3), ...
          'd', ...
          'MarkerSize', mksize, ...
          'MarkerEdgeColor', color, ...
          'MarkerFaceColor', 'w');
end
