function animate_logged_traj(multirotor, zoom_level, speed)
    if nargin < 2
        zoom_level = 0;
    end
    if nargin < 3
        speed = 1;
    end
    
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
    set(fig, 'KeyPressFcn', @Key_Down);
    
    mult_lbl_handle = findobj('Style','text','-and','Tag', 'lblMultirotorFieldValues');
    ee_lbl_handle = findobj('Style','text','-and','Tag', 'lblEndEffectorFieldValues');
    anim_lbl_handle = findobj('Style','text','-and','Tag', 'lblAnimationFieldValues');
    
    set_axis_limits(num_of_zoom_levels, zoom_level, [x(1); y(1); z(1)], x, y, z, min_zoom);
    graphics.PlotMultirotor(multirotor);
    view(3);
    
    dataObjs = findobj(fig,'-property','XData');
    myplot = cell(length(dataObjs), 3);
    for i = 1 : length(dataObjs)
        myplot{i, 1} = dataObjs(i).XData;
        myplot{i, 2} = dataObjs(i).YData;
        myplot{i, 3} = dataObjs(i).ZData;
    end
    
    setup_figure_axes_and_title(fig);

    ind = 1;
    is_paused = false;
    while true

        % Exit the animation if the window is closed
        if ~ishghandle(fig) || ind == length(t)
            break
        end
        
        tic;
        
        curr_pos = [x(ind); y(ind); z(ind)];
        curr_yrp_rad = [yaw(ind); pitch(ind); roll(ind)];
        curr_rpy_deg = rad2deg([roll(ind); pitch(ind); yaw(ind)]);
        curr_time = t(ind);
        
        figure(fig);

        % Draw the robot
        dataObjs = transform_robot(curr_pos, curr_yrp_rad, dataObjs, myplot);

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
            while ind > 1 && current_time - 2 <= t(ind)
                ind = ind - 1;
            end
        elseif key_code == 29 % right arrow key
            while ind < length(t) && current_time + 2 >= t(ind)
                ind = ind + 1;
            end
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

function setup_figure_axes_and_title(fig)
    figure(fig);
    
    % Convert the coordinates to NED
    set(fig.CurrentAxes, 'YDir', 'reverse');
    set(fig.CurrentAxes, 'ZDir', 'reverse');

    xlabel('N');     ylabel('E');     zlabel('D');
    grid on
end

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

function dataObjs = transform_robot(pos, rpy, dataObjs, myplot)
    T = [eul2rotm(rpy', 'ZYX'), pos; 0, 0, 0, 1];
    for i = 1 : length(dataObjs)
        if startsWith(dataObjs(i).Type, 'p')
            data = [myplot{i, 1}, myplot{i, 2}, myplot{i, 3}]';
            data = T * [data; ones(1, length(myplot{i, 1}))];
            set(dataObjs(i),'XData', data(1, :)', 'YData', data(2, :)', 'ZData', data(3, :)');
        elseif startsWith(dataObjs(i).Type, 'l')
            data = [myplot{i, 1}; myplot{i, 2}; myplot{i, 3}];
            data = T * [data; ones(1, length(myplot{i, 1}))];
            set(dataObjs(i),'XData', data(1, :), 'YData', data(2, :), 'ZData', data(3, :));
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
        lim = [minx, minx + step];
    end
    if lim(2) > maxx
        lim = [maxx - step, maxx];
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
end
