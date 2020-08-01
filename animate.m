function animate(multirotor, zoom_level)
    if nargin < 2
        zoom_level = 0;
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

    % Initialization
    
    rotor_pos = zeros([3, 6]);
    for i = 1 : 6
        arm_length = multirotor.Rotors{i}.ArmLength;
        dihedral = multirotor.Rotors{i}.DihedralAngle;
        arm_angle = multirotor.Rotors{i}.ArmAngle;
        rotor_pos(1,i) = arm_length * cosd(dihedral) * cosd(arm_angle);
        rotor_pos(2,i) = arm_length * cosd(dihedral) * sind(arm_angle);
        rotor_pos(3,i) = arm_length * sind(dihedral);
    end

    dir_pos = [multirotor.Rotors{1}.ArmLength * cosd(multirotor.Rotors{1}.DihedralAngle) / 2; 0; 0];

    % set up first frame
    close all; 
    figH = figure('KeyPressFcn',@Key_Down);

    ind = 1;
    is_paused = false;
    while ind <= length(t)

        if is_paused == true
            pause(0.05);
            continue;
        end
        
        tic;
        
        % Exit the animation if the window is closed
        if ~ishghandle(figH)
            break
        end
        
        draw_hex([x(ind), y(ind), z(ind)], [yaw(ind), pitch(ind), roll(ind)], ...
            rotor_pos, dir_pos);

        % Convert the coordinates to NED
        set(gca, 'ydir', 'reverse');
    %    set(gca, 'zdir', 'reverse');

        xlabel('N');     ylabel('E');     zlabel('D');
        
        xlimits = calc_axis_limits(num_of_zoom_levels, zoom_level, x(ind), min(x) - 2, max(x) + 2, min_zoom);
        ylimits = calc_axis_limits(num_of_zoom_levels, zoom_level, y(ind), min(y) - 2, max(y) + 2, min_zoom);
        zlimits = calc_axis_limits(num_of_zoom_levels, zoom_level, z(ind), min(z) - 2, max(z) + 2, min_zoom);
        xlim(xlimits);
        ylim(ylimits);
        zlim(zlimits);

        strtitle = sprintf('Time: %4.1f   X: %4.1f   Y: %4.1f   Z: %4.1f   Roll: %4.1f   Pitch: %4.1f   Yaw: %4.1f', ...
            t(ind), x(ind), y(ind), z(ind), rad2deg(roll(ind)), rad2deg(pitch(ind)), rad2deg(yaw(ind))); 

        title(strtitle);

        grid on
        drawnow;
        exec_time = toc;

        current_time = t(ind);
        while ind < length(t) && current_time + exec_time >= t(ind)
            ind = ind + 1;
            exec_time = toc;
        end

        pause(t(ind) - current_time - exec_time);
        if ind == length(t)
            break;
        end
    end
    
    function Key_Down(src,event)
        key_code = int8(event.Character);
        if key_code == 32
            is_paused = ~is_paused;
        elseif key_code == 43 || key_code == 61
            zoom_level = min(zoom_level + 1, num_of_zoom_levels);
        elseif key_code == 45
            zoom_level = max(zoom_level - 1, 0);
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


function lim = calc_axis_limits(n_levels, level, r_x, minx, maxx, min_lim)
    step = (maxx - minx) * (1 - level / n_levels);
    if step < min_lim
        step = min_lim;
    end
    lim = [r_x - step / 2, r_x + step / 2];
    if lim(1) < minx
        lim = [minx, step];
    end
    if lim(2) > maxx
        lim = [maxx - step, maxx];
    end
end