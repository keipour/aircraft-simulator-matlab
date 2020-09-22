function animate_logged_traj(multirotor, environment, zoom_level, speed, ...
    show_info, show_horizon, fpv_cam, video_fps, video_filename)

    num_of_zoom_levels = 9;
    zoom_level = min(zoom_level, num_of_zoom_levels);
    zoom_level = max(zoom_level, 0);
    min_zoom = 2; % in meters
    
    show_horizon = show_horizon && show_info;
    show_fpv = ~isempty(fpv_cam) && show_info;
    
    is_recording = false;
    video_writer = [];
    if ~isempty(video_filename)
        is_recording = true;
        video_writer = VideoWriter(video_filename, 'Uncompressed AVI');
        % video_writer.Quality = 100;
        if video_fps > 0
            video_writer.FrameRate = video_fps;
        else
            video_writer.FrameRate = 30;
        end
    end
    
    % Load the data from the logger
    [states, t] = logger.GetMeasuredStates();
    [pos, ~] = logger.GetMeasuredPositions();
    axis_limits = [min(pos, [], 1)', max(pos, [], 1)'];
    
    % Set up the first frame
    [fig, form_handles, plot_handles, plot_data, xyz_limits] = support_files.create_frame_figure...
        (multirotor, environment, show_info, show_horizon, show_fpv, is_recording, fpv_cam);
    
    axis_limits(:, 1) = min(xyz_limits(:, 1), axis_limits(:, 1));
    axis_limits(:, 2) = max(xyz_limits(:, 2), axis_limits(:, 2));

    set(fig, 'KeyPressFcn', []);
    set(fig, 'WindowKeyPressFcn', @Key_Down);
    set(fig,'WindowScrollWheelFcn',@Mouse_Scroll);
    
    % This is to restore the rotation capability in R2018b and later which 
    % is disabled by 'WindowScrollWheelFcn'. Needs testing in older
    % versions of MATLAB as well.
    enableDefaultInteractivity(form_handles.axanim);
    
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
        
        % Keep the keyboard hotkeys on even when the figure tools are selected.
        % Note: This is from undocumented MATLAB and may change in releases
        % after R2019b. 
        % Source: http://undocumentedmatlab.com/articles/enabling-user-callbacks-during-zoom-pan
        hManager = uigetmodemanager(fig);
        try
            set(hManager.WindowListenerHandles, 'Enable', 'off');  % HG1
        catch
            [hManager.WindowListenerHandles.Enabled] = deal(false);  % HG2
        end
        set(fig, 'KeyPressFcn', []);
        set(fig, 'WindowKeyPressFcn', @Key_Down);
        
        % Get the current time and state for this iteration
        curr_time = t(ind);
        curr_state = states{ind};
        
        try
            % Draw the frame
            [plot_handles] = support_files.draw_frame(fig, curr_state, ...
                curr_time, plot_handles, plot_data, form_handles, ...
                num_of_zoom_levels, zoom_level, axis_limits, min_zoom, speed, ...
                show_info, show_horizon, show_fpv, fpv_cam);

            if is_recording
                curr_frame = getframe(gcf);
                writeVideo(video_writer, curr_frame);
            end

            drawnow;
        catch ex
            if strcmp(ex.identifier, 'images:imshow:invalidAxes')
                break;
            end
            if is_recording
                break;
            end
            rethrow(ex)
        end
        
        exec_time = toc;
        
        ind = pause_and_update_index(is_paused, t, speed, curr_time, exec_time, ind, video_fps);
    end
    
    if is_recording
        close(video_writer);
        
        % Convert to mp4 if ffmpeg is available
        try
            ret_val = 1;
            if isunix % for Linux
                [ret_val, ~] = system(sprintf('ffmpeg -i %s -y -an -c:v libx264 -crf 0 -preset slow %s', [video_filename '.avi'], [video_filename '.mp4']));
            elseif ispc % for Windows
                [ret_val, ~] = system(sprintf('ffmpeg.exe -i %s -y -an -c:v libx264 -crf 0 -preset slow %s', [video_filename '.avi'], [video_filename '.mp4']));
            end
            if ret_val ~= 0
                warning('ffmpeg command not found or not properly called. The AVI video not converted to MP4');
            end
        catch
            warning('Error converting the AVI video to MP4');
        end
    end
    
    if isfield(form_handles,'fpvfig') && ishghandle(form_handles.fpvfig)
        close(form_handles.fpvfig)
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
            zoom_level = zoom_level - 1;
        elseif key_code == 30 % up key
            speed = min(speed * 2, 16);
        elseif key_code == 31 % down key
            speed = max(speed / 2, 1/32);
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

    function Mouse_Scroll(~, event)
        zoom_scroll_steps = 0.5;
        zoom_level = min(zoom_level - event.VerticalScrollCount * zoom_scroll_steps, num_of_zoom_levels);
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
