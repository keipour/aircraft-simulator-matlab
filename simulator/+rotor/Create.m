function rot = Create()
    rot.InwardAngle = 0;        % in degrees
    rot.SidewardAngle = 0;      % in degrees
    rot.DihedralAngle = 0;      % in degrees
    rot.ArmAngle = 0;           % in degrees
    rot.ArmLength = 0.4;        % in meters
    rot.TimeConstant = 0.01;    % for motor in secs
    rot.ConstantGain = 1;       % for motor
    rot.RPMLimit = 5000;        % for motor
    rot.ThrustConstant = 1.08105e-4;
    rot.TorqueConstant = 0.06e-4;
    rot.MotorMass = 0.10;       % in Kilograms
    rot.ArmMass = 0.150;        % in Kilograms
    rot.RotationDirection = 1;  % -1 for CW, 1 for CCW around Z
                                  % Remember that Z is downward
                               
    rot.R = eye(3);             % Rotation matrix RRB
    rot.Position = zeros(3, 1); % Position of the rotor in B
    rot.MaxrotorSpeedSquared = 0;
    
    rot = rotor.UpdateStructure(rot);
end
