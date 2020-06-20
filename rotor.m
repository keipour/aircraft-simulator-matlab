classdef rotor
    %ROTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        InwardAngle = 0;        % in degrees
        SidewardAngle = 0;      % in degrees
        DihedralAngle = 0;      % in degrees
        ArmAngle                % in degrees
        ArmLength = 0.4;        % in meters
        TimeConstant = 0.01;    % for motor in secs
        ConstantGain = 1;       % for motor
        RPMLimit = 3000;        % for motor
        ThrustConstant = 1.08105e-4;
        TorqueConstant = 0.05;
        MotorMass = 0.10;       % in Kilograms
        ArmMass = 0.150;        % in Kilograms
        
        RotationDirection       % -1 for CW, 1 for CCW around Z
                                % Remember that Z is downward
                                
        R                       % Rotation matrix RRB
        Position                % Position in B
    end
    
    properties(SetAccess=protected, GetAccess=public)
        MaxRotorSpeedSquared    % Maximum rotor speed squared
    end
    
    methods
        function obj = rotor()
            obj = obj.UpdateStructure();
        end
        
        function obj = set.InwardAngle(obj, value)
            obj.InwardAngle = value;
            obj = obj.UpdateStructure();
        end
        
        function obj = set.DihedralAngle(obj, value)
            obj.DihedralAngle = value;
            obj = obj.UpdateStructure();
        end
        
        function obj = set.ArmLength(obj, value)
            obj.ArmLength = value;
            obj = obj.UpdateStructure();
        end
        
        function obj = set.SidewardAngle(obj, value)
            obj.SidewardAngle = value;
            obj = obj.UpdateStructure();
        end

        function obj = set.ArmAngle(obj, value)
            obj.ArmAngle = value;
            obj = obj.UpdateStructure();
        end
        
        function obj = set.RPMLimit(obj, value)
            obj.RPMLimit = value;
            obj = obj.UpdateStructure();
        end
        
        function obj = UpdateStructure(obj)
            obj.R = obj.GetRotationMatrix();
            obj.MaxRotorSpeedSquared = (obj.RPMLimit / 30 * pi).^2;
            obj.Position = obj.GetPosition();
        end
        
        function R = GetRotationMatrix(obj)
            if isempty(obj.ArmAngle)
                R = eye(3);
                return;
            end
            
            mu = deg2rad(obj.ArmAngle);            
            phix = deg2rad(obj.InwardAngle);
            phiy = deg2rad(obj.SidewardAngle);
            
            RotZB1 = [0, 1, 0; 
                      -1, 0, 0; 
                      0, 0, 1];

            RotZB2 = [cos(mu), sin(mu), 0; 
                      -sin(mu), cos(mu), 0;
                      0, 0, 1];
      
            RotZB = RotZB2 * RotZB1;

            RotXp = [1, 0, 0;
                     0, cos(phix), sin(phix);
                     0, -sin(phix), cos(phix)];

            RotYpp = [cos(phiy), 0, -sin(phiy);
                      0, 1, 0;
                      sin(phiy), 0, cos(phiy)];

            R = RotYpp * RotXp * RotZB;
        end
        
        function r = GetPosition(obj)
            rx = obj.ArmLength * cosd(obj.DihedralAngle) * cosd(obj.ArmAngle);
            ry = obj.ArmLength * cosd(obj.DihedralAngle) * sind(obj.ArmAngle);
            rz = -obj.ArmLength * sind(obj.DihedralAngle);
            r = [rx; ry; rz];
        end

        function F = GetThrustForce(obj, rotor_speed_squared)
            rotor_speed_squared = min(rotor_speed_squared, obj.MaxRotorSpeedSquared);
            F = obj.R' * [0; 0; -obj.ThrustConstant * rotor_speed_squared];
        end
        
        function M = GetReactionMoment(obj, rotor_speed_squared)
            rotor_speed_squared = min(rotor_speed_squared, obj.MaxRotorSpeedSquared);
            M = obj.R' * [0; 0; obj.RotationDirection * obj.TorqueConstant ...
                * rotor_speed_squared];
        end
    end
end
