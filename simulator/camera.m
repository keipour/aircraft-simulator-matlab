classdef camera < handle
    properties
        Offset = [0; 0; 0];   % In FRD on the multirotor frame
        ViewAngle = 100;        % in degrees
    end
    
    properties (SetAccess = private, GetAccess = public)
        Axis                    % Axis object to draw the camera frame
        Normal = [1; 0; 0];     % Direction of the camera normal in FRD
        Up = [0; 0; -1];        % Up direction of the camera in FRD
        Roll = 0;               % Camera roll angle in degrees
        Pitch = 0;              % Camera pitch angle in degrees
        Yaw = 0;                % Camera yaw angle in degrees
    end

    %% Public methods
    methods
        function obj = camera()
            obj.UpdateUpNormal();
        end
        
        function SetRPY(obj, rpy)
            obj.Roll = wrapTo180(rpy(1));
            obj.Pitch = wrapTo180(rpy(2));
            obj.Yaw = wrapTo180(rpy(3));
            obj.UpdateUpNormal();
        end
        
        function SetNormalAndUp(obj, normal, up)
            obj.Normal = normal;
            obj.Up = up;
            obj.UpdateRPY();
        end
        
        function Initialize(obj, ax)
            obj.Axis = ax;
            set(ax, 'Xtick', [], 'Ytick', [], 'Ztick', [], 'Box', 'on', ...
                'YDir', 'Reverse', 'ZDir', 'Reverse', 'Visible', 'on');
            camproj(ax, 'perspective');
            axis(ax, 'off');
            axis(ax, 'equal');
%             camlight(ax, 'headlight');
        end
        
        function UpdateState(obj, T)
            pos = T * [obj.Offset; 1];
            normal =  T * [obj.Normal; 0]; 
            up = T * [obj.Up; 0];
            campos(obj.Axis, pos(1 : 3));
            camtarget(obj.Axis, pos(1 : 3) + normal(1 : 3));
            camva(obj.Axis, obj.ViewAngle);
            camup(obj.Axis, up(1 : 3));
        end
        
        function CopyTo(obj, dest_ax)
            fpv_frame = frame2im(getframe(obj.Axis));
            imshow(fpv_frame, 'Parent', dest_ax);
        end
        
    end
    
    methods (Access = private)
        function UpdateUpNormal(obj)
            R = physics.GetRotationMatrixDegrees(obj.Roll, obj.Pitch, obj.Yaw)';
            obj.Up = -R(1:3, 3);
            obj.Normal = R(1:3, 1);
        end
        
        function UpdateRPY(obj)
            R = [obj.Normal, cross(obj.Normal, obj.Up), -obj.Up];
            rpy = physics.GetRPYDegrees(R');
            obj.Roll = rpy(1);
            obj.Pitch = rpy(2);
            obj.Yaw = rpy(3);
        end
    end
end