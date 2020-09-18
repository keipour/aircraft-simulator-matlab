function e = straight_wall(wind, load_texture, x_limit, y_limit)
    
    % Set the defaults:
    if nargin < 1
        wind = [];
    end
    if nargin < 2
        load_texture = [];
    end
    if nargin < 3
        x_limit = [];
    end
    if nargin < 4
        y_limit = [];
    end
    
    e = worlds.empty_world(wind, load_texture, x_limit, y_limit);
    h = e.AddCuboidObject([15; 10; -2.5 - 1e-4], [2; 10; 5], [0; 0; 0]);
    if load_texture
        e.AddTextureToObject(h, ['+worlds' filesep 'textures' filesep 'wall.jpg'], 0.25, 1);
    end
end
