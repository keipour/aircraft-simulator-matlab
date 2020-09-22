%% Visualizer for multirotors
% This file visualizes the environment geometry
% Author: Azarakhsh Keipour (keipour@gmail.com)
% Last updated: September 22, 2020
function [H, xyz_limits] = visualize_environment(e, plot_only)
    
    if nargin < 2
        plot_only = false;
    end

    % Visualization settings
    plot_title = 'Your Cool Environment';
    lighting_on = ~plot_only; % turn on the special lighting

    H = [];
    xyz_limits = [inf(3, 1), -inf(3, 1)];

    for i = 1 : length(e.Objects)
        o = e.Objects{i};
        g = o.Geometry;

        % Extract the limits if it's an object
        if o.Type == 0
            vertices = g.Pose * [
                 g.X/2, -g.Y/2, -g.Z/2, 1; 
                 g.X/2,  g.Y/2, -g.Z/2, 1;
                -g.X/2,  g.Y/2, -g.Z/2, 1;
                -g.X/2, -g.Y/2, -g.Z/2, 1;
                 g.X/2, -g.Y/2,  g.Z/2, 1; 
                 g.X/2,  g.Y/2,  g.Z/2, 1;
                -g.X/2,  g.Y/2,  g.Z/2, 1;
                -g.X/2, -g.Y/2,  g.Z/2, 1]';

            maxv = max(vertices, [], 2);
            minv = min(vertices, [], 2);
            xyz_limits(:, 1) = min(xyz_limits(:, 1), minv(1 : 3));
            xyz_limits(:, 2) = max(xyz_limits(:, 2), maxv(1 : 3));
        end
        
        hold on
        if isempty(o.Texture)
            [~,g_handle] = show(o.Geometry);
            g_handle.EdgeColor = 'none';
            g_handle.FaceColor = o.Color;
        else
            % Inflate it just a bit
            XI = g.X + 1e-4;
            YI = g.Y + 1e-4;
            ZI = g.Z + 1e-4;
            top_left_front =     [-XI / 2;   -YI / 2;   -ZI / 2;   1];
            bottom_left_front =  [-XI / 2;   -YI / 2;    ZI / 2;   1];
            top_right_front =    [-XI / 2;    YI / 2;   -ZI / 2;   1];
            bottom_right_front = [-XI / 2;    YI / 2;    ZI / 2;   1];
            top_left_back =      [ XI / 2;   -YI / 2;   -ZI / 2;   1];
            bottom_left_back =   [ XI / 2;   -YI / 2;    ZI / 2;   1];
            top_right_back =     [ XI / 2;    YI / 2;   -ZI / 2;   1];
            bottom_right_back =  [ XI / 2;    YI / 2;    ZI / 2;   1];
            if o.Type == 0 % cuboid object
                %set(gcf,'Renderer','OpenGL');
                g_handle = [];
                [X, Y, Z] = get_transformed_meshgrid(bottom_right_front, top_left_front, g.Pose);
                g_handle = [g_handle; warp(X, Y, Z, o.Texture.Front)];
                [X, Y, Z] = get_transformed_meshgrid(top_right_back, bottom_left_back, g.Pose);
                g_handle = [g_handle; warp(X, Y, Z, o.Texture.Front)];
                [X, Y, Z] = get_transformed_meshgrid(top_right_back, top_left_front, g.Pose);
                g_handle = [g_handle; warp(X, Y, Z, o.Texture.Top)];
                [X, Y, Z] = get_transformed_meshgrid(bottom_right_back, bottom_left_front, g.Pose);
                g_handle = [g_handle; warp(X, Y, Z, o.Texture.Top)];
                [X, Y, Z] = get_transformed_meshgrid(bottom_right_front, top_right_back, g.Pose);
                g_handle = [g_handle; warp(X, Y, Z, o.Texture.Side)];
                [X, Y, Z] = get_transformed_meshgrid(bottom_left_front, top_left_back, g.Pose);
                g_handle = [g_handle; warp(X, Y, Z, o.Texture.Side)];
            elseif o.Type == 1 % Ground
                g = o.Geometry;
                tlf = g.Pose * top_left_front;
                trb = g.Pose * top_right_back;
                [X, Y, Z] = meshgrid(tlf(1) : g.X / 2 : trb(1), tlf(2) : g.Y / 2 : trb(2), 0 : g.Z / 2 : 0);
                g_handle = warp(X, Y, Z, o.Texture.Top);
            end
        end
        H = [H; g_handle];
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

%% Helper functions

function [X, Y, Z] = get_transformed_meshgrid(pt1, pt2, T)

    % Create the ranges
    x = [pt1(1), pt2(1)];
    y = [pt1(2), pt2(2)];
    z = [pt1(3), pt2(3)];
    
    % Create the meshgrid
    [Xq, Yq, Zq] = meshgrid(x, y, z);
    
    % Transform the meshgrid
    XYZ = [Xq(:) Yq(:) Zq(:) ones(size(Xq(:)))];
    rotXYZ = XYZ * T';
    
    % Reshape the result to the original size
    X = reshape(rotXYZ(:, 1), size(Xq, 1), []);
    Y = reshape(rotXYZ(:, 2), size(Yq, 1), []);
    Z = reshape(rotXYZ(:, 3), size(Zq, 1), []);
    
end
