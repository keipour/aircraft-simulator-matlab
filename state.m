classdef state
    %STATE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Acceleration        = zeros(3, 1);  % Linear acceleration
        EulerDerivative     = zeros(3, 1);  % The derivatives of RPY
        AngularAcceleration = zeros(3, 1);  % Angular acceleration
        Position            = zeros(3, 1);  % Position in global frame
        Velocity            = zeros(3, 1);  % Linear velocity
        RPY                 = zeros(3, 1);  % Roll/Pitch/Yaw
        Omega               = zeros(3, 1);  % Angular velocity
        Force               = zeros(3, 1);  % Total generated force
        Moment              = zeros(3, 1);  % Total generated moment
    end
    
    properties(SetAccess=protected, GetAccess=public)
        R                   % The rotation matrix
    end
    
    methods
        function obj = state()
            obj.R = obj.UpdateRotationMatrix();
        end
        
        function obj = set.RPY(obj, value)
            obj.RPY = value;
            obj = obj.UpdateRotationMatrix();
        end
        
        function obj = UpdateRotationMatrix(obj)
            roll = deg2rad(obj.RPY(1));
            pitch = deg2rad(obj.RPY(2));
            yaw = deg2rad(obj.RPY(3));
            obj.R = angle2dcm(yaw, pitch, roll);
        end
    end
end

