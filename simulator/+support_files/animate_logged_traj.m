function animate_logged_traj(multirotor, environment, zoom_level, speed, ...
    show_info, show_horizon, show_fpv, video_fps, video_filename)

    num_of_zoom_levels = 9;
    zoom_level = min(zoom_level, num_of_zoom_levels);
    zoom_level = max(zoom_level, 0);
    min_zoom = 2; % in meters
    
    show_horizon = show_horizon && show_info;
    show_fpv = show_fpv && show_info;
    
    is_recording = false;
    video_writer = [];
    if ~isempty(video_filename)
        is_recording = true;
        video_writer = VideoWriter(video_filename);
        video_writer.Quality = 100;
        if video_fps > 0
            video_writer.FrameRate = video_fps;
        else
            video_writer.FrameRate = 30;
        end
    end
    
    % Load the data from the logger
    [states, t] = logger.GetMeasuredStates();
    [pos, ~] = logger.GetMeasuredPositions();
    pos_lim = [min(pos, [], 1)', max(pos, [], 1)'];
    
    % Set up the first frame
    [fig, handles, multirotorObjs, multirotor_data, shadowObjs, shadow_data] ...
    = support_files.create_frame_figure(multirotor, environment, show_info, ...
     show_horizon, show_fpv, is_recording);
    
    set(fig, 'WindowKeyPressFcn', @Key_Down);
    %set(fig, 'KeyPressFcn', @Key_Down);
    
    %set_axis_limits(num_of_zoom_levels, zoom_level, [x(1); y(1); z(1)], x, y, z, min_zoom);
    
    if is_recording
        open(video_writer);
    end
    
    ind = 1;
    is_paused = false;
    while true

        tic;

        % Exit the animation if the window is closed
        if ~ishghandle(fig) %|| ind == length(t)
            break
        end
        
        curr_time = t(ind);
        curr_state = states{ind};
        
        try
            % Draw the frame
            [multirotorObjs, shadowObjs] = support_files.draw_frame(fig, curr_state, ...
                curr_time, multirotorObjs, multirotor_data, shadowObjs, shadow_data, ...
                handles, num_of_zoom_levels, zoom_level, pos_lim, min_zoom, speed, ...
                show_info, show_horizon, show_fpv);

            if is_recording
                curr_frame = getframe(gcf);
                writeVideo(video_writer, curr_frame);
            end

            drawnow;
        catch ex
            if strcmp(ex.identifier, 'images:imshow:invalidAxes')
                break;
            end
            rethrow(ex)
        end
        
        exec_time = toc;
        
        ind = pause_and_update_index(is_paused, t, speed, curr_time, exec_time, ind, video_fps);
    end
    
    if is_recording
        close(video_writer);
    end
    
    if isfield(handles,'fpvfig') && ishghandle(handles.fpvfig)
        close(handles.fpvfig)
    end
    
    function Key_Down(~, event)
        key_code = int8(event.Character);
        if ~isnumeric(key_code) || isempty(key_code)
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

function ind = pause_and_update_index(is_paused, t, speed, curr_time, exec_time, ind, video_fps)
    if is_paused == true
        if video_fps > 0
            pause(1 / video_fps);
        else
            pause(0.03);
        end
        return;
    end

    time_forward = exec_time;
    if video_fps > 0
        time_forward = 1 / video_fps;
    end
    
    while ind < length(t) && curr_time + time_forward * speed >= t(ind)
        ind = ind + 1;
    end
    
    pause_time = 1e-6;
    if video_fps > 0
        pause_time = max(1 / video_fps - exec_time, 1e-6);
    end
    pause(pause_time);
end
