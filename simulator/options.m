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

    end
    
end

