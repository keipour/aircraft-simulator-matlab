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
    
    n_rotors = multirotor.NumOfRotors;
    
    % Load the data from the logger

    [pos, t] = logger.GetMeasuredPositions();
    x = pos(:, 1);
    y = pos(:, 2);
    z = pos(:, 3);
    rpy = deg2rad(logger.GetMeasuredRPYs());
    roll = rpy(:, 1);
    pitch = rpy(:, 2);
    yaw = rpy(:, 3);

    % Initialization
    
    rotor_pos = zeros([3, n_rotors]);
    for i = 1 : n_rotors
        arm_length = multirotor.Rotors{i}.ArmLength;
        dihedral = multirotor.Rotors{i}.DihedralAngle;
        arm_angle = multirotor.Rotors{i}.ArmAngle;
        rotor_pos(1,i) = arm_length * cosd(dihedral) * cosd(arm_angle);
        rotor_pos(2,i) = arm_length * cosd(dihedral) * sind(arm_angle);
        rotor_pos(3,i) = arm_length * sind(dihedral);
    end

    % set up first frame
    uif = uifigure('Position', [0 0 300 300]);
    horizon = uiaerohorizon(uif, 'Position', [0 0 300 300]);
    fig = figure('WindowKeyPressFcn',@Key_Down);
    graphics.PlotMultirotor(multirotor);
    view(3);
    
    dataObjs = findobj(fig,'-property','XData');
    myplot = cell(length(dataObjs), 3);
    for i = 1 : length(dataObjs)
        myplot{i, 1} = dataObjs(i).XData;
        myplot{i, 2} = dataObjs(i).YData;
        myplot{i, 3} = dataObjs(i).ZData;
    end
    
    ind = 1;
    is_paused = false;
    while true

        tic;
        
        % Exit the animation if the window is closed
        if ~ishghandle(fig)
            break
        end
        
        T = [eul2rotm([yaw(ind); pitch(ind); roll(ind)]', 'ZYX'), ...
            [x(ind); y(ind); z(ind)]; 0, 0, 0, 1];
        figure(fig);
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
        
        horizon.Roll = rad2deg(roll(ind));
        horizon.Pitch = rad2deg(pitch(ind));
        
%    dir_pos = [multirotor.Rotors{1}.ArmLength * cosd(multirotor.Rotors{1}.DihedralAngle) / 2; 0; 0];
%         figure(fig);
%         draw_hex([x(ind), y(ind), z(ind)], [yaw(ind), pitch(ind), roll(ind)], ...
%             rotor_pos, dir_pos);

        % Convert the coordinates to NED
        set(fig.CurrentAxes, 'ydir', 'reverse');
        set(gca, 'zdir', 'reverse');

        xlabel('N');     ylabel('E');     zlabel('D');
        
        [xlimits, ylimits, zlimits] = calc_all_axis_limits...
            (num_of_zoom_levels, zoom_level, [x(ind); y(ind); z(ind)],...
            x, y, z, min_zoom);
        xlim(xlimits);
        ylim(ylimits);
        zlim(zlimits);
        
        strtitle = sprintf('Time: %4.1f  Zoom: %d  Speed: %4.2fx\nXYZ: (%4.1f, %4.1f, %4.1f)  RPY: (%4.1f, %4.1f, %4.1f)', ...
            t(ind), zoom_level, speed, x(ind), y(ind), z(ind), rad2deg(roll(ind)), rad2deg(pitch(ind)), rad2deg(yaw(ind))); 

        title(strtitle);

        grid on
        drawnow;
        exec_time = toc;

        if is_paused == true
            pause(0.05);
            continue;
        end
        
        current_time = t(ind);
        while ind < length(t) && current_time + exec_time * speed >= t(ind)
            ind = ind + 1;
            exec_time = toc;
        end

        pause(t(ind) - current_time - exec_time * speed);
    end
    
    try
        delete(uif);
    catch
    end
    
    function Key_Down(src,event)
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

function draw_hex(pos, eul, rotor_pos, dir_pos)
    rotm = eul2rotm(eul, 'ZYX');
    
    rotor_pos = rotm * rotor_pos;
    rotor_pos(1, :) = rotor_pos(1, :) + pos(1);
    rotor_pos(2, :) = rotor_pos(2, :) + pos(2);
    rotor_pos(3, :) = rotor_pos(3, :) + pos(3);
    
    hold off
    
    dir_pos(:) = rotm * dir_pos(:);
    dir_pos(1) = pos(1) + dir_pos(1);
    dir_pos(2) = pos(2) + dir_pos(2);
    dir_pos(3) = pos(3) + dir_pos(3);

    plot3([pos(1), dir_pos(1)], [pos(2), dir_pos(2)], [pos(3), dir_pos(3)], 'g')

    for i = 1 : 6
       hold on
       plot3([pos(1), rotor_pos(1, i)], [pos(2), rotor_pos(2,i)], [pos(3), rotor_pos(3,i)], 'b')
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
