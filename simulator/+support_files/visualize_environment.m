%% Visualizer for multirotors
% This file visualizes the environment geometry
% Author: Azarakhsh Keipour (keipour@gmail.com)
% Last updated: August 11, 2020
function visualize_environment(e, plot_only)
    
    if nargin < 2
        plot_only = false;
    end

    % Visualization settings
    ground_color = [0.5, 0.85, 0.85];
    obstacle_color = [0, 0, 0.8];
    plot_title = 'Your Cool Environment';
    lighting_on = ~plot_only; % turn on the special lighting

    for i = 1 : length(e.Objects)
        o = e.Objects{i};
        hold on
        [~,patchObj] = show(o.Geometry);
        patchObj.EdgeColor = 'none';
        if o.Type == 0
            patchObj.FaceColor = obstacle_color;
        elseif o.Type == 1
            patchObj.FaceColor = ground_color;
        end
    end

    % Make the plot more presentable

    if plot_only == false
        % Rotate the axes for better visualization
        set(gca, 'Xdir', 'reverse')
        set(gca, 'Zdir', 'reverse')

        % Equalize the axes scales
        axis equal;

        % Add title and axis labels
        xlabel('N');
        ylabel('E');
        zlabel('D');
        title(plot_title);
    end

    % Change lighting
    if lighting_on
        camlight
        lighting gouraud %phong
    end
    
    hold off
end

