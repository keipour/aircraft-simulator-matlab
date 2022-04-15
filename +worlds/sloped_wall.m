function e = sloped_wall(wall_angle, wind, load_texture, x_limit, y_limit)
    
    % Set the defaults:
    if nargin < 2
        wind = [];
    end
    if nargin < 3
        load_texture = [];
    end
    if nargin < 4
        x_limit = [];
    end
    if nargin < 5
        y_limit = [];
    end
    
    e = worlds.empty_world(wind, load_texture, x_limit, y_limit);
    h = e.AddCuboidObject([15; 10; -2.5 - 1e-4], [2; 10; 5], [0; wall_angle; 0]);
    if load_texture
        e.AddTextureToObject(h, ['+worlds' filesep 'textures' filesep 'wall.jpg'], 0.25, 1);
    end
end
