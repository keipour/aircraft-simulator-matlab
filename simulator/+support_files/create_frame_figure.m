function [fig, label_handles, multirotorObjs, multirotor_data, shadowObjs, shadow_data] ...
    = create_frame_figure(multirotor, environment, show_info)
    

    % Set up the first frame
    if show_info
        fig = openfig('+support_files/animation_gui.fig');
    else
        fig = figure;
    end
    
    label_handles.mult1 = findobj('Style','text','-and','Tag', 'lblMultirotorFieldValues1');
    label_handles.mult2 = findobj('Style','text','-and','Tag', 'lblMultirotorFieldValues2');
    label_handles.mult3 = findobj('Style','text','-and','Tag', 'lblMultirotorFieldValues3');
    label_handles.ee1 = findobj('Style','text','-and','Tag', 'lblEndEffectorFieldValues1');
    label_handles.ee2 = findobj('Style','text','-and','Tag', 'lblEndEffectorFieldValues2');
    label_handles.ee3 = findobj('Style','text','-and','Tag', 'lblEndEffectorFieldValues3');
    label_handles.anim1 = findobj('Style','text','-and','Tag', 'lblAnimationFieldValues1');
    label_handles.anim2 = findobj('Style','text','-and','Tag', 'lblAnimationFieldValues2');
    
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

    xlabel('N');     ylabel('E');     zlabel('D');
    
    fig.CurrentAxes.YDir = 'Reverse';
    fig.CurrentAxes.ZDir = 'Reverse';
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
