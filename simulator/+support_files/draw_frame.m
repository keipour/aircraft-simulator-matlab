function [plot_handles] = draw_frame(fig, rbt, curr_state, curr_time, plot_handles, ...
    plot_data, form_handles, num_of_zoom_levels, zoom_level, axis_limits, ...
    min_zoom, speed, show_info, show_horizon, show_fpv, fpv_cam)

    leave_mark_on_wall = false;

    figure(fig);
    
    % Get the current transform
    rpy = deg2rad(curr_state.RPY);
    T = [physics.GetRotationMatrixRadians(rpy(1), rpy(2), rpy(3))', curr_state.Position; 0, 0, 0, 1];
    
    Tr = cell(rbt.NumOfRotors, 1);
    for i = 1 : rbt.NumOfRotors
        inw = curr_state.RotorInwardAngles(i);
        side = curr_state.RotorSidewardAngles(i);
        arm_angle = rbt.Rotors{i}.ArmAngle;
        T_pos = [eye(3), rbt.Rotors{i}.Position; 0, 0, 0, 1];
        T_posT = [eye(3), -rbt.Rotors{i}.Position; 0, 0, 0, 1];
        curr_Rbr = [rbt.Rotors{i}.CalcRotorationMatrix(arm_angle, inw, side), zeros(3, 1); 0, 0, 0, 1];
        init_Rbr = [rbt.Rotors{i}.R_BR, zeros(3, 1); 0, 0, 0, 1];
        Tr{i} = T * T_pos * curr_Rbr * init_Rbr' * T_posT;
    end

    % Draw the robot
    transform_robot(T, plot_handles.Multirotor, plot_data.Multirotor);

    % Draw the rotors
    transform_rotors(Tr, plot_handles.Rotors, plot_data.Rotors);

    % Draw the shadow
    transform_shadow(T, plot_handles.Shadow, plot_data.Shadow);

    set_frame_limits(num_of_zoom_levels, zoom_level, curr_state.Position, axis_limits, min_zoom);

    if show_horizon
        form_handles.horizon.update(rpy(3), rpy(2), rpy(1));
    end

    if leave_mark_on_wall && curr_state.InCollision && curr_state.ForceSensor(3) > 4.8
        hold on
        plot3(curr_state.EndEffectorPosition(1), curr_state.EndEffectorPosition(2), ...
            curr_state.EndEffectorPosition(3), 'w*');
    end
    
    if show_fpv
        copy_data_object_points(plot_handles.FPVMultirotor, plot_handles.Multirotor);
        copy_data_object_points(plot_handles.FPVShadow, plot_handles.Shadow);
        fpv_cam.UpdateState(T);
        fpv_cam.CopyTo(form_handles.axfpv);
        if leave_mark_on_wall && curr_state.InCollision && curr_state.ForceSensor(3) > 4.8
            hold(form_handles.axfpvfig, 'on');
            plot3(form_handles.axfpvfig, curr_state.EndEffectorPosition(1), curr_state.EndEffectorPosition(2), ...
                curr_state.EndEffectorPosition(3), 'w*');
        end
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

function transform_object(T, o_handle, o_data)
    if startsWith(o_handle.Type, 'p') % patch
        data = [o_data{1}, o_data{2}, o_data{3}]';
        data = T * [data; ones(1, length(o_data{1}))];
        set(o_handle,'XData', data(1, :)', 'YData', data(2, :)', 'ZData', data(3, :)');
    elseif startsWith(o_handle.Type, 'l') % line
        data = [o_data{1}; o_data{2}; o_data{3}];
        data = T * [data; ones(1, length(o_data{1}))];
        set(o_handle,'XData', data(1, :), 'YData', data(2, :), 'ZData', data(3, :));
    elseif startsWith(o_handle.Type, 's') % surface
        data = [o_data{1}(:), o_data{2}(:), o_data{3}(:)]';
        data = T * [data; ones(1, numel(o_data{1}))]; 
        X = reshape(data(1, :), size(o_data{1}, 1), []);
        Y = reshape(data(2, :), size(o_data{2}, 1), []);
        Z = reshape(data(3, :), size(o_data{3}, 1), []);
        set(o_handle, 'XData', X, 'YData', Y, 'ZData', Z);
    end
end

function transform_robot(T, obj_handles, robot_data)
    for i = 1 : length(obj_handles)
        transform_object(T, obj_handles(i), robot_data(i, :));
    end
end

function transform_rotors(Tr, obj_handles, rotor_data)
    for i = 1 : length(obj_handles)
        for j = 1 : length(obj_handles{i})
            transform_object(Tr{i}, obj_handles{i}(j), rotor_data(i, j, :));
        end
    end
end

function transform_shadow(T, obj_handles, shadow_data)
    for i = 1 : length(obj_handles)
        if startsWith(obj_handles(i).Type, 'p') % patch
            data = [shadow_data{i, 1}, shadow_data{i, 2}, shadow_data{i, 3}]';
            data = T * [data; ones(1, length(shadow_data{i, 1}))];
            set(obj_handles(i),'XData', data(1, :)', 'YData', data(2, :)', 'ZData', -0.001*ones(size(data(3, :)')));
        elseif startsWith(obj_handles(i).Type, 'l') % line
            data = [shadow_data{i, 1}; shadow_data{i, 2}; shadow_data{i, 3}];
            data = T * [data; ones(1, length(shadow_data{i, 1}))];
            set(obj_handles(i),'XData', data(1, :), 'YData', data(2, :), 'ZData', -0.001*ones(size(data(3, :)')));
        elseif startsWith(obj_handles(i).Type, 's') % surface
            data = [shadow_data{i, 1}(:), shadow_data{i, 2}(:), shadow_data{i, 3}(:)]';
            data = T * [data; ones(1, numel(shadow_data{i, 1}))]; 
            X = reshape(data(1, :), size(shadow_data{i, 1}, 1), []);
            Y = reshape(data(2, :), size(shadow_data{i, 2}, 1), []);
            Z = reshape(data(3, :), size(shadow_data{i, 3}, 1), []);
            set(obj_handles(i), 'XData', X, 'YData', Y, 'ZData',  -0.001*ones(size(Z)));
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

    expand = 1;
    limits(:, 1) = limits(:, 1) - expand;
    limits(:, 2) = limits(:, 2) + expand;
    
    stepx = calc_single_axis_size(n_levels, level, limits(1, 1), limits(1, 2), min_lim);
    stepy = calc_single_axis_size(n_levels, level, limits(2, 1), limits(2, 2), min_lim);
    stepz = calc_single_axis_size(n_levels, level, limits(3, 1), limits(3, 2), min_lim);
    
    step = max([stepx, stepy, stepz]) + expand / 2;
    
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

    a_str1 = sprintf('%0.3f\n\n%0.1f', curr_time, zoom_level);
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
