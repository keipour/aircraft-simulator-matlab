function animate_logged_traj(multirotor, environment, zoom_level, speed)

    num_of_zoom_levels = 9;
    zoom_level = min(zoom_level, num_of_zoom_levels);
    zoom_level = max(zoom_level, 0);
    min_zoom = 2; % in meters
    
    % Load the data from the logger
    [states, t] = logger.GetMeasuredStates();
    [pos, ~] = logger.GetMeasuredPositions();
    pos_lim = [min(pos, [], 1)', max(pos, [], 1)'];
    
    % Set up the first frame
    uif = uifigure('Position', [0 0 300 300]);
    horizon = uiaerohorizon(uif, 'Position', [0 0 300 300]);

    [fig, lh, multirotorObjs, multirotor_data, shadowObjs, shadow_data] ...
    = support_files.create_frame_figure(multirotor, environment);
    
    set(fig, 'WindowKeyPressFcn', @Key_Down);
    %set(fig, 'KeyPressFcn', @Key_Down);
    
    %set_axis_limits(num_of_zoom_levels, zoom_level, [x(1); y(1); z(1)], x, y, z, min_zoom);
    
    ind = 1;
    is_paused = false;
    while true

        tic;

        % Exit the animation if the window is closed
        if ~ishghandle(fig) || ind == length(t)
            break
        end
        
        curr_time = t(ind);
        curr_state = states{ind};
        
        % Draw the frame
        [multirotorObjs, shadowObjs] = support_files.draw_frame(fig, curr_state, ...
            curr_time, multirotorObjs, multirotor_data, shadowObjs, shadow_data, ...
            lh, num_of_zoom_levels, zoom_level, pos_lim, min_zoom, speed);
        
        horizon.Roll = curr_state.RPY(1);
        horizon.Pitch = curr_state.RPY(2);
        
        drawnow;
        exec_time = toc;
        
        ind = pause_and_update_index(is_paused, t, speed, curr_time, exec_time, ind);
    end
    
    try
        delete(uif);
    catch
    end
    
    function Key_Down(~, event)
        key_code = int8(event.Character);
        if ~isnumeric(key_code) %|| isempty(key_code)
            return;
        end
        
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
            while ind > 1 && curr_time - 2 * speed <= t(ind)
                ind = ind - 1;
            end
            curr_time = t(ind);
        elseif key_code == 29 % right arrow key
            while ind < length(t) && curr_time + 2 * speed >= t(ind)
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
