classdef options
    
    properties(Constant)
        %% Graphics Settings
        
        % Artificial Horizon settings
        AH_GroundColor = [0.549 0.3882 0.2157]; % Some nice light green color: [0.8 1.0 0.8];
        AH_SkyColor = [0.0 0.6824 1.0]; % Some nice light sky blue color: [0.8 0.898 1.0];
        AH_EdgeColor = 'w'; %[0.2 0.2 0.2];
        AH_ReticleColor = [0.9686 0.9412 0.1412]; % Some nice cyan color: [0.301 0.745 0.933];
        AH_ReticleType = '-.-'; % Options are: '-.-' | '-o-' | '+' | 'v' | 'w'
        AH_ReticleWidth = 3;
        AH_LineWidth = 1;
        AH_FontSize = 8;
        AH_FontWeight = 'Demi'; % Options are: 'Light' | 'Demi' | 'Bold'

        % Multirotor Visualization settings
        MV_PayloadHeight = 0.1; % in meters
        MV_AxisArrowSize = 0.3; % in meters
        MV_PlotTitle = '';%'Your Cool Multirotor';
        MV_ShowArmLabels = true; % Options are: true | false
        MV_MotorHeight = 0.02; % in meters -- set to zero if don't want motors
        MV_MotorRadius = 0.02; % in meters -- set to zero if don't want motors
        MV_AddLighting = true; % Options are: true | false
        MV_ShowRotorAxes = true; % Options are: true | false
        MV_MotorColor = 'black';
        MV_RotorColorCW = [0.4, 0.4, 1];
        MV_RotorColorCCW = [0.4, 1, 0.4];
        MV_RotorOpacity = 0.85;
        MV_ShowPlotAxes = 'off'; % Options are: 'on' | 'off'
        
        % Dynamic Manipulability settings
        DM_CrossSectionPoints = 1e5;    % Default: 1e5
        DM_LateralThrustMonteCarloPoints = 5e3;    % Default: 5e3
        DM_LateralThrustColor = 'b';
        DM_CrossSectionColor = 'b';
        DM_CrossSectionZFromZero = true; % Options are: true | false
        DM_CrossSectionSubplotRows = 3;
        DM_CrossSectionSubplotCols = 3;
        DM_ConvexHullFaceColor = 'cyan';
        DM_ConvexHullFaceAlpha = 0.8; % Options are: 0..1
        DM_ConvexHullEdgeColor = [0 0 0]; % Options are: [0 0 0] | color name | RGB triplet | 'none'
        DM_ConvexHullLineStyle = '-'; % Options are: '-' | '--' | ':' | '-.' | 'none'
        DM_DrawAccelerationOmniSphere = true; % Options are: true | false
        DM_DrawAccelerationConvexHull = true; % Options are: true | false
        DM_DrawAngularAccelerationConvexHull = false; % Options are: true | false
        DM_DrawAccelerationCrossSectionX = false; % Options are: true | false
        DM_DrawAccelerationCrossSectionY = false; % Options are: true | false
        DM_DrawAccelerationCrossSectionZ = true; % Options are: true | false
        DM_DrawAngularAccelerationCrossSectionX = false; % Options are: true | false
        DM_DrawAngularAccelerationCrossSectionY = false; % Options are: true | false
        DM_DrawAngularAccelerationCrossSectionZ = false;   % Options are: true | false   
        
        % Simulation settings
        SS_ShowWaitbar = true; % Options are: true | false
        
    end    
end

