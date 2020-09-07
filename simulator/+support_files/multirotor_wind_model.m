classdef multirotor_wind_model < handle
    properties
        LateralEffectivenessFactor = 0.67;  % How much of the area is filled
        BaseEffectivenessFactor = 0.8;      % How much of the area is filled
    end
    
    properties (SetAccess=protected, GetAccess=public)
        CylinderRadius = 0;             % Cylinder radius for effective wind area
        CylinderHeight = 0;             % Cylinder height for effective wind area
        LateralArea = 0;                % Maximum effective lateral area (1/2*pi*r*h*alpha)
        BaseArea = 0;                   % Maximum effective base area (pi*r^2*beta)
    end
    
    methods
        function set.LateralEffectivenessFactor(obj, value)
            obj.LateralEffectivenessFactor = value;
            obj.UpdateAreas();
        end
            
        function set.BaseEffectivenessFactor(obj, value)
            obj.BaseEffectivenessFactor = value;
            obj.UpdateAreas();
        end
        
        function Update(obj, multirotor)
            % The multirotor is approximated by a cylinder for calculation
            % of the effective area under the wind pressure. The cylinder
            % height is the payload diameter and the cylinder radius is the
            % longest rotor arm. 
            obj.CylinderHeight = 2 * multirotor.PayloadRadius;
            obj.CylinderRadius = 0;
            for i = 1 : multirotor.NumOfRotors
                obj.CylinderRadius = max(obj.CylinderRadius, multirotor.Rotors{i}.ArmLength);
            end
            obj.UpdateAreas();
        end
        
        function CopyFrom(obj, wm)
            obj.LateralEffectivenessFactor = wm.LateralEffectivenessFactor;
            obj.BaseEffectivenessFactor = wm.BaseEffectivenessFactor;
            obj.CylinderHeight = wm.CylinderHeight;
            obj.CylinderRadius = wm.CylinderRadius;
            obj.LateralArea = wm.LateralArea;
            obj.BaseArea = wm.BaseArea;
        end
        
        function area = CalcualteEffectiveArea(obj, wind_dir, rpy)
            if all(wind_dir == 0)
                area = 0;
                return;
            end
            wind_vec = wind_dir / norm(wind_dir);
            RBI = physics.GetRotationMatrixDegrees(rpy(1), rpy(2), rpy(3));
            z_axis = RBI(1:3, 3);
            cos_eff = dot(z_axis, wind_vec);
            sin_eff = sqrt(1 - cos_eff^2);
            area = cos_eff * obj.BaseArea + sin_eff * obj.LateralArea;
        end
    end
    
    methods (Access = private)
        function UpdateAreas(obj)
            obj.LateralArea = 1/2 * pi * obj.CylinderRadius * obj.CylinderHeight * obj.LateralEffectivenessFactor;
            obj.BaseArea = pi * (obj.CylinderRadius.^2) * obj.BaseEffectivenessFactor;
        end
    end
end