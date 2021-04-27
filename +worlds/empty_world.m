function e = empty_world(wind, load_texture, x_limit, y_limit)
    
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
    
    if isempty(wind)
        wind = zeros(3, 1);
    end
    if isempty(load_texture)
        load_texture = true;
    end
    if isempty(x_limit)
        x_limit = [-10; 30];
    end
    if isempty(y_limit)
        y_limit = [-10; 30];
    end
    
    e = environment;
    %h = e.AddGroundPlane(x_limit, y_limit);
    if load_texture
        e.AddTextureToObject(h, ['+worlds' filesep 'textures' filesep 'grass.jpg'], 0.12, 8);
    end
    e.AverageWind = wind;
end
