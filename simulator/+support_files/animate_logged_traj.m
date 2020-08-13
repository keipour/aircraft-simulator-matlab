function animate_logged_traj(multirotor, environment, zoom_level, speed)

    num_of_zoom_levels = 5;
    zoom_level = min(zoom_level, num_of_zoom_levels);
    zoom_level = max(zoom_level, 0);
    min_zoom = 2; % in meters
    
    % Load the data from the logger
    [pos, t] = logger.GetMeasuredPositions();
    x = pos(:, 1);
    y = pos(:, 2);
    z = pos(:, 3);
    rpy = deg2rad(logger.GetMeasuredRPYs());
    roll = rpy(:, 1);
    pitch = rpy(:, 2);
    yaw = rpy(:, 3);

    % Set up the first frame
    uif = uifigure('Position', [0 0 300 300]);
    horizon = uiaerohorizon(uif, 'Position', [0 0 300 300]);
    fig = openfig('+support_files/animation_gui.fig');
    set(fig, 'WindowKeyPressFcn', @Key_Down);
    %set(fig, 'KeyPressFcn', @Key_Down);
    
    mult_lbl_handle = findobj('Style','text','-and','Tag', 'lblMultirotorFieldValues');
    ee_lbl_handle = findobj('Style','text','-and','Tag', 'lblEndEffectorFieldValues');
    anim_lbl_handle = findobj('Style','text','-and','Tag', 'lblAnimationFieldValues');
    
    %set_axis_limits(num_of_zoom_levels, zoom_level, [x(1); y(1); z(1)], x, y, z, min_zoom);
    
    % Draw and save the multirotor
    multirotorObjs = graphics.PlotMultirotor(multirotor);
    multirotor_data = cell(length(multirotorObjs), 3);
    for i = 1 : length(multirotorObjs)
        multirotor_data{i, 1} = multirotorObjs(i).XData;
        multirotor_data{i, 2} = multirotorObjs(i).YData;
        multirotor_data{i, 3} = multirotorObjs(i).ZData;
    end
    
    % Draw the multirotor shadow
    hold on
    shadowObjs = circlePlane3D([0, 0, 0], [0; 0; 1], multirotor.PayloadRadius * 2, 0.2, 'black', 0.5);
    shadow_data = cell(length(shadowObjs), 3);
    for i = 1 : length(shadowObjs)
        shadow_data{i, 1} = shadowObjs(i).XData;
        shadow_data{i, 2} = shadowObjs(i).YData;
        shadow_data{i, 3} = shadowObjs(i).ZData;
    end
    
    % Draw the environment
    graphics.PlotEnvironment(environment);

    view(3);

    figure(fig);
    xlabel('N');     ylabel('E');     zlabel('D');
    grid on

    ind = 1;
    is_paused = false;
    while true

        % Exit the animation if the window is closed
        if ~ishghandle(fig) %|| ind == length(t)
            break
        end
        
        tic;
        
        curr_pos = [x(ind); y(ind); z(ind)];
        curr_yrp_rad = [yaw(ind); pitch(ind); roll(ind)];
        curr_rpy_deg = rad2deg([roll(ind); pitch(ind); yaw(ind)]);
        curr_time = t(ind);
        
        figure(fig);

        % Draw the robot
        multirotorObjs = transform_robot(curr_pos, curr_yrp_rad, multirotorObjs, multirotor_data);

        % Draw the shadow
        shadowObjs = transform_shadow(curr_pos, curr_yrp_rad, shadowObjs, shadow_data);

        horizon.Roll = curr_rpy_deg(1);
        horizon.Pitch = curr_rpy_deg(2);
        
        set_axis_limits(num_of_zoom_levels, zoom_level, curr_pos, x, y, z, min_zoom);
        show_status(mult_lbl_handle, ee_lbl_handle, anim_lbl_handle, ...
            curr_time, zoom_level, speed, curr_pos, curr_rpy_deg);
        
        drawnow;
        exec_time = toc;
        
        ind = pause_and_update_index(is_paused, t, speed, curr_time, exec_time, ind);
    end
    
    try
        delete(uif);
    catch
    end
    
    function Key_Down(~,event)
        key_code = int8(event.Character);
        if key_code == 32 % space key
            is_paused = ~is_paused;
        elseif key_code == 43 || key_code == 61 % + key
            zoom_level = min(zoom_level + 1, num_of_zoom_levels);
        elseif key_code == 45 % - key
            zoom_level = max(zoom_level - 1, 0);
        elseif key_code == 30 % up key
            speed = min(speed * 2, 16);
        elseif key_code == 31 % down key
            speed = max(speed / 2, 1/16);
        elseif key_code == 28 % left arrow key
            while ind > 1 && curr_time - 2 <= t(ind)
                ind = ind - 1;
            end
            curr_time = t(ind);
        elseif key_code == 29 % right arrow key
            while ind < length(t) && curr_time + 2 >= t(ind)
                ind = ind + 1;
            end
            curr_time = t(ind);
        elseif key_code == 42 || key_code == 46 % . or *
            speed = 1;
        elseif key_code >= 48 && key_code <= 48 + num_of_zoom_levels % numbers
            zoom_level = double(key_code) - 48;
        elseif key_code == 27 % ESC key
            delete(gcbf);
        end
    end
end

%% Helper functions

function ind = pause_and_update_index(is_paused, t, speed, curr_time, exec_time, ind)
    if is_paused == true
        pause(0.03);
        return;
    end

    while ind < length(t) && curr_time + exec_time * speed >= t(ind)
        ind = ind + 1;
    end

    pause_time = t(ind) - curr_time - exec_time * speed;
    pause(pause_time);
end

function show_status(m_lbl, e_lbl, a_lbl, curr_time, zoom_level, speed, curr_pos, curr_rpy_deg)
    m_str = sprintf('%0.2f\n\n%0.2f\n\n%0.2f\n\n%0.2f%c\n\n%0.2f%c\n\n%0.2f%c', ...
        curr_pos(1), curr_pos(2), curr_pos(3), curr_rpy_deg(1), char(176), ...
        curr_rpy_deg(2), char(176), curr_rpy_deg(3), char(176));

    e_str = sprintf('-\n\n-\n\n-\n\n');

    a_str = sprintf('%0.2f\n\n%d\n\n%0.2fx', curr_time, zoom_level, speed);

    %title(strtitle);
    
    set(m_lbl, 'String', m_str);
    set(e_lbl, 'String', e_str);
    set(a_lbl, 'String', a_str);
end

function set_axis_limits(num_of_zoom_levels, zoom_level, curr_pos, x, y, z, min_zoom)
    [xlimits, ylimits, zlimits] = calc_all_axis_limits(num_of_zoom_levels, ...
        zoom_level, curr_pos, x, y, z, min_zoom);
    xlim(xlimits);
    ylim(ylimits);
    zlim(zlimits);
end

function dataObjs = transform_robot(pos, rpy, dataObjs, multirotor_data)
    T = [eul2rotm(rpy', 'ZYX'), pos; 0, 0, 0, 1];
    for i = 1 : length(dataObjs)
        if startsWith(dataObjs(i).Type, 'p') % patch
            data = [multirotor_data{i, 1}, multirotor_data{i, 2}, multirotor_data{i, 3}]';
            data = T * [data; ones(1, length(multirotor_data{i, 1}))];
            set(dataObjs(i),'XData', data(1, :)', 'YData', data(2, :)', 'ZData', data(3, :)');
        elseif startsWith(dataObjs(i).Type, 'l') % line
            data = [multirotor_data{i, 1}; multirotor_data{i, 2}; multirotor_data{i, 3}];
            data = T * [data; ones(1, length(multirotor_data{i, 1}))];
            set(dataObjs(i),'XData', data(1, :), 'YData', data(2, :), 'ZData', data(3, :));
        elseif startsWith(dataObjs(i).Type, 's') % surface
            data = [multirotor_data{i, 1}(:), multirotor_data{i, 2}(:), multirotor_data{i, 3}(:)]';
            data = T * [data; ones(1, numel(multirotor_data{i, 1}))]; 
            X = reshape(data(1, :), size(multirotor_data{i, 1}, 1), []);
            Y = reshape(data(2, :), size(multirotor_data{i, 2}, 1), []);
            Z = reshape(data(3, :), size(multirotor_data{i, 3}, 1), []);
            set(dataObjs(i), 'XData', X, 'YData', Y, 'ZData', Z);
        end
    end
end

function dataObjs = transform_shadow(pos, rpy, dataObjs, shadow_data)
    T = [eul2rotm(rpy', 'ZYX'), pos; 0, 0, 0, 1];
    for i = 1 : length(dataObjs)
        if startsWith(dataObjs(i).Type, 'p') % patch
            data = [shadow_data{i, 1}, shadow_data{i, 2}, shadow_data{i, 3}]';
            data = T * [data; ones(1, length(shadow_data{i, 1}))];
            set(dataObjs(i),'XData', data(1, :)', 'YData', data(2, :)', 'ZData', -0.001*ones(size(data(3, :)')));
        elseif startsWith(dataObjs(i).Type, 'l') % line
            data = [shadow_data{i, 1}; shadow_data{i, 2}; shadow_data{i, 3}];
            data = T * [data; ones(1, length(shadow_data{i, 1}))];
            set(dataObjs(i),'XData', data(1, :), 'YData', data(2, :), 'ZData', -0.001*ones(size(data(3, :)')));
        elseif startsWith(dataObjs(i).Type, 's') % surface
            data = [shadow_data{i, 1}(:), shadow_data{i, 2}(:), shadow_data{i, 3}(:)]';
            data = T * [data; ones(1, numel(shadow_data{i, 1}))]; 
            X = reshape(data(1, :), size(shadow_data{i, 1}, 1), []);
            Y = reshape(data(2, :), size(shadow_data{i, 2}, 1), []);
            Z = reshape(data(3, :), size(shadow_data{i, 3}, 1), []);
            set(dataObjs(i), 'XData', X, 'YData', Y, 'ZData',  -0.001*ones(size(Z)));
        end
    end
end

function step = calc_single_axis_size(n_levels, level, minx, maxx, min_lim)
    step = (maxx - minx) * (1 - level / n_levels);
    if step < min_lim
        step = min_lim;
    end
end

function lim = calc_single_axis_limits(r_x, minx, maxx, step)
    lim = [r_x - step / 2, r_x + step / 2];
    if lim(1) < minx
        lim(1) = minx;
    end
    if lim(2) > maxx
        lim(2) = maxx;
    end
    new_step = lim(2) - lim(1);
    if new_step < step
        diff = step - new_step;
        lim = [lim(1) - diff / 2, lim(2) + diff / 2];
    end
end

function [limx, limy, limz] = calc_all_axis_limits(n_levels, level, pos, x, y, z, min_lim)
    expand = 2;
    
    minx = min(x) - expand; maxx = max(x) + expand;
    miny = min(y) - expand; maxy = max(y) + expand;
    minz = min(z) - expand; maxz = max(z) + expand;
    
    stepx = calc_single_axis_size(n_levels, level, minx, maxx, min_lim);
    stepy = calc_single_axis_size(n_levels, level, miny, maxy, min_lim);
    stepz = calc_single_axis_size(n_levels, level, minz, maxz, min_lim);
    
    step = max([stepx, stepy, stepz]);
    
    limx = calc_single_axis_limits(pos(1), minx, maxx, step);
    limy = calc_single_axis_limits(pos(2), miny, maxy, step);
    limz = calc_single_axis_limits(pos(3), minz, maxz, step);
    
    % Correction for ground
    ground_thickness = 0.001;
    if limz(2) > ground_thickness
        limz = [-step + ground_thickness, ground_thickness];
    end
end

%% Draw a 3-D circle
% Downloaded from https://www.mathworks.com/matlabcentral/fileexchange/37879-circle-plane-in-3d
% With some modifications and bug fixes
function H = circlePlane3D( center, normal, radious, theintv, color, alpha)
    %CIRCLEPLANE3D Summary of this function goes here
    %--------------------------------------------------------------------------
    %Generate a circle plane in 3D with the given center and radious
    %The plane is defined by the normal vector
    %theintv is the interval theta which allow you to control your polygon
    %shape
    % Example:,
    %
    %   circlePlane3D([0 0 0], [1 -1 2], 5, 0.2, 1, [0 0 1], '-'); 
    %   circlePlane3D([3 3 -3],[0 1 1], 3, 0.1, 1, 'y', '-');
    %   
    %   Cheng-Yuan Wu <ieda_wind@hotmail.com>
    %   Version 1.00
    %   Aug, 2012
    %--------------------------------------------------------------------------

    %generate circle polygon
    t = 0:theintv:2*pi;
    x = radious*cos(t);
    y = radious*sin(t);
    z = zeros(size(x));
    %compute rotate theta and axis
    zaxis = [0 0 1];
    normal = normal/norm(normal);
    ang = acos(dot(zaxis,normal));
    axis = cross(zaxis, normal)/norm(cross(zaxis, normal));
    if any(isnan(axis))
        axis = [1 0 0 ];
    end
    % A skew symmetric representation of the normalized axis
    axis_skewed = [ 0 -axis(3) axis(2) ; axis(3) 0 -axis(1) ; -axis(2) axis(1) 0]; 
    % Rodrigues formula for the rotation matrix 
    R = eye(3) + sin(ang)*axis_skewed + (1-cos(ang))*axis_skewed*axis_skewed;
    fx = R(1,1)*x + R(1,2)*y + R(1,3)*z;
    fy = R(2,1)*x + R(2,2)*y + R(2,3)*z;
    fz = R(3,1)*x + R(3,2)*y + R(3,3)*z;
    %translate center
    fx = fx+center(1);
    fy = fy+center(2);
    fz = fz+center(3);
    H = fill3(fx, fy, fz, color, 'FaceAlpha', alpha);
    H.EdgeColor = 'none';
end
