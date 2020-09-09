classdef rotor < handle

    properties
        InwardAngle = 0;                        % in degrees
        SidewardAngle = 0;                      % in degrees
        DihedralAngle = 0;                      % in degrees
        ArmAngle = 0;                           % in degrees
        ArmLength = 0.4;                        % in meters

        TimeConstant = 0.01;                    % for motor in secs
        ConstantGain = 1;                       % for motor
        Kv = 475;                               % in RPM/V (for KDE3510XF-475)
        BatteryVoltage = 22.2;                  % in Volts (for 6-cell lipo)
        RPMLimit = (475) * (22.2);              % Maximum RPM for the motor
        ThrustConstant = 1.08105e-4 / 5;    
        TorqueConstant = (1.08105e-4 / 5) * 0.05;
        PropellerMass = 0.015;                  % For T-Motor 14" * 4.8 propellers
        MotorMass = (0.015) + 0.175;            % in Kilograms (for KDE3510XF-475)
        ESCMass = 0.084;                        % in Kilograms (for KDE-UAS35HVC)
        ArmMass = 0.10 + (0.084);               % in Kilograms
        RotationDirection = 1;                  % -1 for CW, 1 for CCW around Z
                                                % Remember that Z is downward

        R_BR = eye(3);                          % Rotation matrix R_BR
        Position = zeros(3, 1);                 % Position of the rotor in B
        MaxrotorSpeedSquared = 0;
    end
    
    methods
        function obj = rotor(rot)
            if nargin < 1
                obj.UpdateStructure();
            else
                obj = rotor();
                obj.CopyFrom(rot);
            end
        end

        function M = GetReactionMoment(obj, rotor_speed_squared)
            rotor_speed_squared = obj.LimitRotorSpeed(rotor_speed_squared);
            M = obj.R_BR * [0; 0; obj.RotationDirection * obj.TorqueConstant ...
                * rotor_speed_squared];
        end

        function F = GetThrustForce(obj, rotor_speed_squared)
            rotor_speed_squared = obj.LimitRotorSpeed(rotor_speed_squared);
            F = obj.R_BR * [0; 0; -obj.ThrustConstant * rotor_speed_squared];
        end

        function [rotor_speed_squared, saturated] = LimitRotorSpeed(obj, rotor_speed_squared)
            flag = false;

            if rotor_speed_squared > obj.MaxrotorSpeedSquared
                rotor_speed_squared = obj.MaxrotorSpeedSquared;
                flag = true;
            end
            if rotor_speed_squared < 0
                rotor_speed_squared = 0;
                flag = true;
            end

            if nargout > 1
                saturated = flag;
            end
        end
        
        function set.ArmAngle(obj, value)
            obj.ArmAngle = value;
            obj.UpdateStructure();
        end

        function set.ArmLength(obj, value)
            obj.ArmLength = value;
            obj.UpdateStructure();
        end

        function set.DihedralAngle(obj, value)
            obj.DihedralAngle = value;
            obj.UpdateStructure();
        end

        function set.InwardAngle(obj, value)
            obj.InwardAngle = value;
            obj.UpdateStructure();
        end

        function set.RPMLimit(obj, value)
            obj.RPMLimit = value;
            obj.UpdateStructure();
        end

        function set.SidewardAngle(obj, value)
            obj.SidewardAngle = value;
            obj.UpdateStructure();
        end

        function UpdateStructure(obj)
            obj.R_BR = obj.CalcRotorationMatrix();
            obj.MaxrotorSpeedSquared = (obj.RPMLimit / 30 * pi).^2;
            obj.Position = obj.GetPosition();
        end
        
        function CopyFrom(obj, rot)
            obj.InwardAngle = rot.InwardAngle;
            obj.SidewardAngle = rot.SidewardAngle;
            obj.DihedralAngle = rot.DihedralAngle;
            obj.ArmAngle = rot.ArmAngle;
            obj.ArmLength = rot.ArmLength;
            obj.TimeConstant = rot.TimeConstant;
            obj.ConstantGain = rot.ConstantGain;
            obj.Kv = rot.Kv;
            obj.BatteryVoltage = rot.BatteryVoltage;
            obj.RPMLimit = rot.RPMLimit;
            obj.ThrustConstant = rot.ThrustConstant;
            obj.TorqueConstant = rot.TorqueConstant;
            obj.PropellerMass = rot.PropellerMass;
            obj.MotorMass = rot.MotorMass;
            obj.ESCMass = rot.ESCMass;
            obj.ArmMass = rot.ArmMass;
            obj.RotationDirection = rot.RotationDirection;
            obj.R_BR = rot.R_BR;
            obj.Position = rot.Position;
            obj.MaxrotorSpeedSquared = rot.MaxrotorSpeedSquared;
        end
    end
    
    methods (Access = private)

        function r = GetPosition(obj)
            rx = obj.ArmLength * cosd(obj.DihedralAngle) * cosd(obj.ArmAngle);
            ry = obj.ArmLength * cosd(obj.DihedralAngle) * sind(obj.ArmAngle);
            rz = -obj.ArmLength * sind(obj.DihedralAngle);
            r = [rx; ry; rz];
        end
        
        function R_BR = CalcRotorationMatrix(obj)
            if isempty(obj.ArmAngle)
                R_BR = eye(3);
                return;
            end

            rotorZB = rotz(obj.ArmAngle) * rotz(90);
            rotorXp = rotx(obj.InwardAngle);
            rotorYpp = roty(obj.SidewardAngle);

            R_BR = rotorZB * rotorXp * rotorYpp;
        end

    end
end