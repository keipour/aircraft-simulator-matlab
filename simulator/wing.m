classdef wing < handle

    properties
        Direction = [0; 1; 0];
        Length = 1; % in meters
        SurfaceArea = 0.22; % in m^2
    end
    
    properties (GetAccess = public, SetAccess = private)
        R_BR      % Rotation matrix R_BR
    end
    
    methods
        function obj = wing(wng)
            if nargin < 1
                obj.UpdateStructure();
            else
                obj = wing();
                obj.CopyFrom(wng);
            end
        end
        
        function UpdateStructure(obj)
            obj.R_BR = obj.CalcRotorationMatrix();
        end
        
        function CopyFrom(obj, rot)
            obj.InwardAngle = rot.InwardAngle;
        end
        
        function SetPosition(obj, position)
            rbr = obj.R_BR;
            obj.ArmLength = norm(position);
            obj.DihedralAngle = asind(-position(3) / obj.ArmLength);
            obj.ArmAngle = atan2d(position(2), position(1));
            obj.SetR_BR(rbr);
        end

    end

    methods (Static)
        function R_BR = CalcRotorationMatrix(arm_angle, inward_angle, sideward_angle)
            rotorZB = rotz(arm_angle + 90);
            rotorXp = rotx(inward_angle);
            rotorYpp = roty(sideward_angle);

            R_BR = rotorZB * rotorXp * rotorYpp;
        end
    end

end
