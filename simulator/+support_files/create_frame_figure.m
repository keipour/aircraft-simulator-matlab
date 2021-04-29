function [fig, form_handles, plot_handles, plot_data, xyz_limits] = create_frame_figure...
    (rbt, env, show_info, show_horizon, show_fpv, is_recording, fpv_cam)
    

    % Set up the first frame
    if show_info
        fig = openfig('+support_files/animation_gui.fig');
    else
        fig = figure;
    end
    
    form_handles.mult1 = findobj(fig, 'Tag', 'lblMultirotorFieldValues1');
    form_handles.mult2 = findobj(fig, 'Tag', 'lblMultirotorFieldValues2');
    form_handles.mult3 = findobj(fig, 'Tag', 'lblMultirotorFieldValues3');
    form_handles.ee1 = findobj(fig, 'Tag', 'lblEndEffectorFieldValues1');
    form_handles.ee2 = findobj(fig, 'Tag', 'lblEndEffectorFieldValues2');
    form_handles.ee3 = findobj(fig, 'Tag', 'lblEndEffectorFieldValues3');
    form_handles.anim1 = findobj(fig, 'Tag', 'lblAnimationFieldValues1');
    form_handles.anim2 = findobj(fig, 'Tag', 'lblAnimationFieldValues2');
    form_handles.axhorizon = findobj(fig, 'Tag', 'figHorizon');
    form_handles.axanim = findobj(fig, 'Tag', 'figAnimation');
    form_handles.axfpv = findobj(fig, 'Tag', 'figFPV');
    form_handles.hotkeyspanel = findobj(fig, 'Tag', 'pnlHotkeys');
    
    if show_fpv
        form_handles.fpvfig = figure('Position', [0 0 200 200]);
        form_handles.axfpvfig = axes(form_handles.fpvfig);
        fpv_cam.Initialize(form_handles.axfpvfig);
        set(form_handles.fpvfig, 'Visible', 'off')
    end

    if ~isempty(form_handles.axfpv)
        axis(form_handles.axfpv, 'off');
    end
    
    if ~isempty(form_handles.axhorizon)
        axis(form_handles.axhorizon, 'off');
    end
    
    if ~isempty(form_handles.axanim)
        axes(form_handles.axanim);
    end
    
    [plot_handles, plot_data, xyz_limits] = create_all_objects(rbt, env);

    if show_fpv
        plot_handles.FPVMultirotor = copyobj(plot_handles.Multirotor, form_handles.axfpvfig);
        plot_handles.FPVShadow = copyobj(plot_handles.Shadow, form_handles.axfpvfig);
        plot_handles.FPVEnvironment = copyobj(plot_handles.Environment, form_handles.axfpvfig);
    end
    
    view(3);

    xlabel('N');     ylabel('E');     zlabel('D');
    
    if ~show_info
        form_handles.axanim = gca;
        fig.CurrentAxes.YDir = 'Reverse';
        fig.CurrentAxes.ZDir = 'Reverse';
    else
        if is_recording
            form_handles.hotkeyspanel.Visible = 'Off';
        end
    end
    
    if show_horizon
        form_handles.horizon = support_files.artificial_horizon('Axes', form_handles.axhorizon, ...
            'ReticleType', options.AH_ReticleType, 'ReticleColor', options.AH_ReticleColor, ...
            'EdgeColor', options.AH_EdgeColor, 'GroundColor', options.AH_GroundColor, ...
            'SkyColor', options.AH_SkyColor, 'ReticleWidth', options.AH_ReticleWidth, ...
            'LineWidth', options.AH_LineWidth, 'FontSize', options.AH_FontSize, ...
            'FontWeight', options.AH_FontWeight);
    end
    
    hpnlEndEffector = findobj(fig, 'Tag', 'pnlEndEffector');
    if rbt.HasEndEffector()
        set(hpnlEndEffector, 'Visible', 'on');
    else
        set(hpnlEndEffector, 'Visible', 'off');
    end
end

%% Helper functions

function [plot_handles, plot_data, xyz_limits] = create_all_objects(rbt, env)

    % Draw and save the robot and the rotors
    [plot_handles.Multirotor, plot_handles.Rotors] = graphics.PlotMultirotor(rbt);
    plot_data.Multirotor = cell(length(plot_handles.Multirotor), 3);
    for i = 1 : length(plot_handles.Multirotor)
        plot_data.Multirotor{i, 1} = plot_handles.Multirotor(i).XData;
        plot_data.Multirotor{i, 2} = plot_handles.Multirotor(i).YData;
        plot_data.Multirotor{i, 3} = plot_handles.Multirotor(i).ZData;
    end
    if ~isempty(plot_handles.Rotors)
        plot_data.Rotors = cell(length(plot_handles.Rotors), length(plot_handles.Rotors{1}), 3);
        for i = 1 : length(plot_handles.Rotors)
            for j = 1 : length(plot_handles.Rotors{i})
                plot_data.Rotors{i, j, 1} = plot_handles.Rotors{i}(j).XData;
                plot_data.Rotors{i, j, 2} = plot_handles.Rotors{i}(j).YData;
                plot_data.Rotors{i, j, 3} = plot_handles.Rotors{i}(j).ZData;
            end
        end
    end
    
    % Draw the robot shadow
    hold on
    plot_handles.Shadow = circlePlane3D([0, 0, 0], [0; 0; 1], rbt.PayloadRadius * 2, 0.2, 'black', 0.5);
    plot_data.Shadow = cell(length(plot_handles.Shadow), 3);
    for i = 1 : length(plot_handles.Shadow)
        plot_data.Shadow{i, 1} = plot_handles.Shadow(i).XData;
        plot_data.Shadow{i, 2} = plot_handles.Shadow(i).YData;
        plot_data.Shadow{i, 3} = plot_handles.Shadow(i).ZData;
    end
    
    % Draw the environment
    [plot_handles.Environment, xyz_limits] = graphics.PlotEnvironment(env);
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
