function [plot_handles] = draw_frame(fig, curr_state, curr_time, plot_handles, ...
    plot_data, form_handles, num_of_zoom_levels, zoom_level, axis_limits, ...
    min_zoom, speed, show_info, show_horizon, show_fpv, fpv_cam)

    figure(fig);
    
    % Get the current transform
    rpy = deg2rad(curr_state.RPY);
    T = [physics.GetRotationMatrixRadians(rpy(1), rpy(2), rpy(3))', curr_state.Position; 0, 0, 0, 1];

    % Draw the robot
    transform_robot(T, plot_handles.Multirotor, plot_data.Multirotor);

    % Draw the shadow
    transform_shadow(T, plot_handles.Shadow, plot_data.Shadow);

    set_frame_limits(num_of_zoom_levels, zoom_level, curr_state.Position, axis_limits, min_zoom);

    if show_horizon
        form_handles.horizon.update(rpy(3), rpy(2), rpy(1));
    end

    if show_fpv
        copy_data_object_points(plot_handles.FPVMultirotor, plot_handles.Multirotor);
        copy_data_object_points(plot_handles.FPVShadow, plot_handles.Shadow);
        fpv_cam.UpdateState(T);
        fpv_cam.CopyTo(form_handles.axfpv);
    end

    if show_info
        update_frame_labels(form_handles, curr_time, zoom_level, speed, curr_state);
    end
    
end

%% Helper functions

function copy_data_object_points(destObjs, srcObjs)
    for i = 1 : length(srcObjs)
        set(destObjs(i), 'XData', srcObjs(i).XData, 'YData', srcObjs(i).YData, 'ZData', srcObjs(i).ZData);
    end    
end


function transform_robot(T, dataObjs, multirotor_data)
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

function transform_shadow(T, dataObjs, shadow_data)
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

function set_frame_limits(num_of_zoom_levels, zoom_level, curr_pos, limits, min_zoom)

    [xlimits, ylimits, zlimits] = calc_all_axis_limits(num_of_zoom_levels, ...
        zoom_level, curr_pos, limits, min_zoom);
    xlim(xlimits);
    ylim(ylimits);
    zlim(zlimits);
end

function step = calc_single_axis_size(n_levels, level, minx, maxx, min_lim)
    step = (maxx - minx - min_lim) * (1 - level / n_levels) + min_lim;
end

function lim = calc_single_axis_limits(r_x, minx, maxx, step)
    if step + 1e-5 > (maxx - minx)
        center = (minx + maxx) / 2;
        lim = [center - step / 2, center + step / 2];
        return;
    end

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

function [limx, limy, limz] = calc_all_axis_limits(n_levels, level, pos, limits, min_lim)

    expand = 3;
    limits(:, 1) = limits(:, 1) - expand;
    limits(:, 2) = limits(:, 2) + expand;
    
    stepx = calc_single_axis_size(n_levels, level, limits(1, 1), limits(1, 2), min_lim);
    stepy = calc_single_axis_size(n_levels, level, limits(2, 1), limits(2, 2), min_lim);
    stepz = calc_single_axis_size(n_levels, level, limits(3, 1), limits(3, 2), min_lim);
    
    step = max([stepx, stepy, stepz]);
    
    limx = calc_single_axis_limits(pos(1), limits(1, 1), limits(1, 2), step);
    limy = calc_single_axis_limits(pos(2), limits(2, 1), limits(2, 2), step);
    limz = calc_single_axis_limits(pos(3), limits(3, 1), limits(3, 2), step);
    
    % Correction for ground
    ground_thickness = 0.001;
    if limz(2) > ground_thickness
        limz = [-step + ground_thickness, ground_thickness];
    end
end

function update_frame_labels(form_handles, curr_time, zoom_level, speed, curr_state)

    m_str1 = sprintf('%0.3f\n\n%0.3f\n\n%0.3f\n\n%0.3f%c\n\n%0.3f%c\n\n%0.3f%c', ...
        curr_state.Position(1), curr_state.Position(2), curr_state.Position(3), ...
        curr_state.RPY(1), char(176), curr_state.RPY(2), char(176), curr_state.RPY(3), char(176));

    m_str2 = sprintf('%0.3f\n\n%0.3f\n\n%0.3f\n\n%0.3f\n\n%0.3f\n\n%0.3f', ...
        curr_state.Velocity(1), curr_state.Velocity(2), curr_state.Velocity(3), ...
        curr_state.Omega(1), curr_state.Omega(2), curr_state.Omega(3));

    m_str3 = sprintf('%0.3f\n\n%0.3f\n\n%0.3f\n\n%0.3f\n\n%0.3f\n\n%0.3f', ...
        curr_state.Acceleration(1), curr_state.Acceleration(2), curr_state.Acceleration(3),...
        curr_state.AngularAcceleration(1), curr_state.AngularAcceleration(2), curr_state.AngularAcceleration(3));

    e_str1 = sprintf('%0.3f\n\n%0.3f\n\n%0.3f', curr_state.EndEffectorPosition(1), ...
        curr_state.EndEffectorPosition(2), curr_state.EndEffectorPosition(3));
    e_str2 = sprintf('%0.3f\n\n%0.3f\n\n%0.3f', curr_state.EndEffectorVelocity(1), ...
        curr_state.EndEffectorVelocity(2), curr_state.EndEffectorVelocity(3));
    e_str3 = sprintf('%0.3f\n\n%0.3f\n\n%0.3f', curr_state.ForceSensor(1), ...
        curr_state.ForceSensor(2), curr_state.ForceSensor(3));

    a_str1 = sprintf('%0.3f\n\n%d', curr_time, zoom_level);
    a_str2 = sprintf('%0.2fx\n\n%d', speed, curr_state.InCollision);

    %title(strtitle);
    
    set(form_handles.mult1, 'String', m_str1);
    set(form_handles.mult2, 'String', m_str2);
    set(form_handles.mult3, 'String', m_str3);
    set(form_handles.ee1, 'String', e_str1);
    set(form_handles.ee2, 'String', e_str2);
    set(form_handles.ee3, 'String', e_str3);
    set(form_handles.anim1, 'String', a_str1);
    set(form_handles.anim2, 'String', a_str2);
    
end
