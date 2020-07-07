function rot = Create()
    rot.InwardAngle = 0;                % in degrees
    rot.SidewardAngle = 0;              % in degrees
    rot.DihedralAngle = 0;              % in degrees
    rot.ArmAngle = 0;                   % in degrees
    rot.ArmLength = 0.4;                % in meters

    rot.TimeConstant = 0.01;            % for motor in secs
    rot.ConstantGain = 1;               % for motor
    rot.Kv = 475;                       % in RPM/V (for KDE3510XF-475)
    rot.BatteryVoltage = 22.2;          % in Volts (for 6-cell lipo)
    rot.RPMLimit = rot.Kv * 22.2;       % for motor
    rot.ThrustConstant = 1.08105e-4 / 5;    
    rot.TorqueConstant = 0.06e-4 / 5;
    rot.PropellerMass = 0.015;          % For T-Motor 14" * 4.8 propellers
    rot.MotorMass = rot.PropellerMass + 0.175;  % in Kilograms (for KDE3510XF-475)
    rot.ESCMass = 0.084;                % in Kilograms (for KDE-UAS35HVC)
    rot.ArmMass = 0.10 + rot.ESCMass;   % in Kilograms
    rot.RotationDirection = 1;          % -1 for CW, 1 for CCW around Z
                                        % Remember that Z is downward
                               
    rot.R_BR = eye(3);                  % Rotation matrix R_BR
    rot.Position = zeros(3, 1);         % Position of the rotor in B
    rot.MaxrotorSpeedSquared = 0;
    
    rot = rotor.UpdateStructure(rot);
end
