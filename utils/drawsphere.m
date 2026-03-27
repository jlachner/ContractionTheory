function drawsphere(ax_handle)
% DRAWSPHERE Plots a transparent sphere of radius pi at the origin
%   ax_handle: handle to the target axes

    if nargin < 1 || ~ishandle(ax_handle) || ~strcmp(get(ax_handle, 'type'), 'axes')
        error('drawsphere requires a valid axes handle as input.');
    end

    % Create unit sphere
    [XS, YS, ZS] = sphere(40);  % Increase 40 for smoother sphere

    % Scale to radius pi
    radius = pi;
    XS = radius * XS;
    YS = radius * YS;
    ZS = radius * ZS;

    % Plot sphere
    surf(ax_handle, XS, YS, ZS, ...
         'FaceColor', 'none', ...
         'EdgeColor', 'k', ...
         'LineStyle', '-', ...
         'LineWidth', 0.5);

end
