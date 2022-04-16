function e = pole_with_wire(wind, load_texture, x_limit, y_limit)
    
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
    
    shape_x = 14.1;
    bottom = -2;
    top = -4;
    left = 9;
    right = 11;
    border_w = 0.3;
    e = worlds.empty_world(wind, load_texture, x_limit, y_limit);
    h1 = e.AddCuboidObject([shape_x; (left + right) / 2; bottom / 2 - 1e-4], [border_w; border_w; -bottom], [0; 0; 0]);
    h2 = e.AddCuboidObject([shape_x; (left + right) / 2; top], [border_w; right - left; border_w], [0; 0; 0]);
    h3 = e.AddCuboidObject([shape_x; (left + right) / 2; bottom], [border_w; right - left; border_w], [0; 0; 0]);
    h4 = e.AddCuboidObject([shape_x; right; (top + bottom) / 2], [border_w; border_w; abs(top - bottom) + border_w], [0; 0; 0]);
    h5 = e.AddCuboidObject([shape_x; left; (top + bottom) / 2], [border_w; border_w; abs(top - bottom) + border_w], [0; 0; 0]);
    e.AddCuboidObject([shape_x + border_w / 2 - 0.005; (left + right) / 2; (top + bottom) / 2], [0.01; right - left; abs(top - bottom) + border_w], [0; 0; 0], [1, 0.95, 0.85]);

    wire1_y = left + border_w;
    wire2_y = left + 1;
    wire_x = shape_x - border_w / 4;
    wire_w = 0.05;
    wire_l = 1.2;
    wire_z = top + wire_l / 2;
    wire_color = [0.7; 0.2; 0];
    wire1_ctr = [wire_x; wire1_y; wire_z];
    wire2_ctr = [wire_x; wire2_y; wire_z];
    e = draw_straight_wire(e, wire1_ctr, [0; 0; 0], wire_w, wire_l, wire_color);
    e = draw_straight_wire(e, wire2_ctr, [0; 0; 0], wire_w, wire_l, wire_color);
    center = [wire_x; (wire1_y + wire2_y) / 2; wire_z + wire_l / 2];
    radius = abs(wire1_y - wire2_y) / 2;
    e = draw_curved_wire(e, center, radius, wire_w, 0, 180, 10, wire_color);

    wire1_y = left + 0.75;
    wire2_y = right + 0.5;
    wire_x = shape_x;
    wire_l = 0.5;
    wire_z = top + wire_l / 2;
    wire_color = [0.4; 0.6; 0.2];
    wire1_ctr = [wire_x; wire1_y; wire_z];
    e = draw_straight_wire(e, wire1_ctr, [0; 0; 0], wire_w, wire_l, wire_color);
    center = [wire_x; (wire1_y + wire2_y) / 2; wire_z + wire_l / 2];
    radius = abs(wire1_y - wire2_y) / 2;
    e = draw_curved_wire(e, center, radius, wire_w, 60, 180, 5, wire_color);

    wire1_y = left - 1.5;
    wire2_y = right - border_w / 2 + 0.05;
    wire_x = shape_x + border_w / 4;
    wire_l = 0;
    wire_z = top + wire_l / 2;
    wire_color = [0.3; 0.4; 0.5];
    center = [wire_x; (wire1_y + wire2_y) / 2; wire_z + wire_l / 2];
    radius = abs(wire1_y - wire2_y) / 2;
    e = draw_curved_wire(e, center, radius, wire_w, 0, 100, 5, wire_color);

    if load_texture
        e.AddTextureToObject(h1, ['+worlds' filesep 'textures' filesep 'wood.jpg'], 0.25, 1);
        e.AddTextureToObject(h2, ['+worlds' filesep 'textures' filesep 'wood.jpg'], 0.25, 1);
        e.AddTextureToObject(h3, ['+worlds' filesep 'textures' filesep 'wood.jpg'], 0.25, 1);
        e.AddTextureToObject(h4, ['+worlds' filesep 'textures' filesep 'wood.jpg'], 0.25, 1);
        e.AddTextureToObject(h5, ['+worlds' filesep 'textures' filesep 'wood.jpg'], 0.25, 1);
    end
end

function e = draw_straight_wire(e, center, rpy, diameter, length, color)
    e.AddCuboidObject(center, [diameter; diameter; length], rpy, color);
end

function e = draw_curved_wire(e, center, curve_radius, wire_diameter, start_angle, end_angle, steps_deg, color)
    last_point = center + curve_radius * [0; cosd(start_angle); sind(start_angle)];
    for deg = start_angle + steps_deg : steps_deg : end_angle
        new_point = center + curve_radius * [0; cosd(deg); sind(deg)];
        new_ctr = (last_point + new_point) / 2;
        roll = deg - steps_deg / 2;
        length = norm(last_point - new_point);
        e.AddCuboidObject(new_ctr, [wire_diameter; wire_diameter; length], [roll; 0; 0], color);
        last_point = new_point;
    end
end