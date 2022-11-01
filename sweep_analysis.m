%% Prepare the environment
clear all
close all
addpath('simulator');

set(groot,'defaultFigureVisible','off') % turns off all figures

%% Define the hardware architecture
mult = robots.tilted_hex(true);

%% Define resoloution of sweep

rp_step = 1; % in degrees
rp_start = -20;
rp_end = 20;
rp_size = size(rp_start:rp_step:rp_end, 2);

y_step = 90; % in degrees
y_start = 0;
y_end = 360;
y_size = size(y_start:y_step:y_end, 2);


wind_step = 2; % in newtons
wind_start = -2; % max wind force on earth is 1.0e+03*2.0603 N
wind_end = 2;
wind_size = size(wind_start:wind_step:wind_end, 2);


%% Define the current state
position = [0; 0; 0]; % position has no effect on manipulability
velocity = [0; 0; 0]; % in m/s
omega = [0; 0; 0]; % in rad/s
 

% %% Sweep rp
% 
% omni_radius_2d = zeros(rp_size, rp_size);
% [i, j] = deal(1);
% for roll = rp_start:rp_step:rp_end
%     for pitch = rp_start:rp_step:rp_end
%         rpy = [roll; pitch; 0]; % Roll, Pitch and Yaw in degrees
%         mult.SetInitialState(position, velocity, rpy, omega);
%         wind_force = [0; 0; 0]; % in N
%         % Analyze the dynamic manipulability
%         result = mult.AnalyzeDynamicManipulability(wind_force);
%         result.AccelerationOmni
%         omni_radius_2d(i, j) = result.AccelerationOmni;
%         fprintf('%f percent completed!\n', ((i - 1) * rp_size + j) / (rp_size*rp_size) * 100)
%         j = j + 1;
%     end
%     j = 1;
%     i = i + 1;
% end




%% Sweep rp and wind force
% omni_radius_6d = zeros(rp_size, rp_size, y_size, wind_size, wind_size, wind_size);
omni_radius_5d = zeros(rp_size, rp_size, wind_size, wind_size, wind_size);
% omni_radius_3d = zeros(rp_size, rp_size, y_size);
omni_radius_2d = zeros(rp_size, rp_size);
[i, j, l, m, n] = deal(1);
for roll = rp_start:rp_step:rp_end
   for pitch = rp_start:rp_step:rp_end
%        for yaw = y_start:y_step:y_end
           rpy = [roll; pitch; 0]; % Roll, Pitch and Yaw in degrees
           mult.SetInitialState(position, velocity, rpy, omega);
           for x = wind_start:wind_step:wind_end
               for y = wind_start:wind_step:wind_end
                   for z = wind_start:wind_step:wind_end
                       % Define the external forces
                       wind_force = [x; y; z]; % in N
%                        wind_force = [0; 0; 0]; % in N
                       % Analyze the dynamic manipulability
                       result = mult.AnalyzeDynamicManipulability(wind_force);
%                        omni_radius_6d(i, j, k, l, m, n) = result.AccelerationOmni;
                       result.AccelerationOmni
                       omni_radius_5d(i, j, l, m, n) = result.AccelerationOmni;
                       
                       progress = ((i - 1) *rp_size*wind_size*wind_size*wind_size + (j - 1) *wind_size*wind_size*wind_size + (l - 1) * ...
                                    wind_size*wind_size + (m - 1) * wind_size + n) / (rp_size*rp_size*wind_size*wind_size*wind_size) * 100;
                       fprintf('[roll = %f, pitch = %f, x = %f, y = %f, z = %f] \n radius = %f \nProgress: %f percent completed!\n', ...
                                roll, pitch, x, y, z, result.AccelerationOmni, progress);
                       
                       n = n + 1;
                   end
                   n = 1;
                   m = m + 1;
               end
               n = 1;
               m = 1;
               l = l + 1;
           end
%            k = k + 1;
%        end
       n = 1;
       m = 1;
       l = 1;
       j = j + 1;
   end
   n = 1;
   m = 1;
   l = 1;
   j = 1;
   i = i + 1;
   close all
end





