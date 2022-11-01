clear all
close all
clc

mat = load('omni_radius_2d_rp_swept.mat');

roll = mat.roll;
pitch = mat.pitch;
omni = mat.omni_radius_2d;

xx = zeros(size(omni, 1)*size(omni, 2), 1);
yy = zeros(size(omni, 1)*size(omni, 2), 1);
zz = zeros(size(omni, 1)*size(omni, 2), 1);

for i = 1:size(omni, 1)
    for j = 1:size(omni, 2)
        xx((i-1)*size(omni, 2) + j) = roll(i);
        yy((i-1)*size(omni, 2) + j) = pitch(j);
        zz((i-1)*size(omni, 2) + j) = omni(i, j);
    end
end

x = xx(zz~=0);
y = yy(zz~=0);
z = zz(zz~=0);

fit5 = fittype('a5 * x ^ 5 + b5 * y ^ 5 + a4 * x ^ 4 + b4 * y ^ 4 + a3 * x ^ 3 + b3 * y ^ 3 + a2 * x ^ 2 + b2 * y ^ 2 + a1 * x ^ 1 + b1 * y ^ 1 + a0 + b0 + ab * x * y + a2b * x ^ 2 * y + ab2 * x * y ^ 2+ a3b * x ^ 3 * y + ab3 * x * y ^ 3 + a3b2 * x ^ 3 * y ^ 2 + a2b3 * x ^ 2 * y ^ 3 + a4b2 * x ^ 4 * y ^ 2 + a2b4 * x ^ 2 * y ^ 4 + a4b3 * x ^ 4 * y ^ 3 + a3b4 * x ^ 3 * y ^ 4 + a4b * x ^ 4 * y + ab4 * x * y ^ 4',...
'dependent',{'z'},'independent',{'x', 'y'},...
'coefficients',{'a5','b5', 'a4','b4', 'a3','b3', 'a2','b2', 'a1','b1', 'a0','b0', 'ab','a2b','ab2','a3b','ab3', 'a3b2', 'a2b3', 'a4b2', 'a2b4', ...
'a4b3','a3b4', 'a4b', 'ab4'});

fit4 = fittype('a4 * x ^ 4 + b4 * y ^ 4 + a3 * x ^ 3 + b3 * y ^ 3 + a2 * x ^ 2 + b2 * y ^ 2 + a1 * x ^ 1 + b1 * y ^ 1 + a0 + b0 + ab * x * y + a2b * x ^ 2 * y + ab2 * x * y ^ 2+ a3b * x ^ 3 * y + ab3 * x * y ^ 3 + a3b2 * x ^ 3 * y ^ 2 + a2b3 * x ^ 2 * y ^ 3',...
'dependent',{'z'},'independent',{'x', 'y'},...
'coefficients',{'a4','b4', 'a3','b3', 'a2','b2', 'a1','b1', 'a0','b0', 'ab','a2b','ab2','a3b','ab3', 'a3b2', 'a2b3'});

exp_poly4 = fittype('a6 * exp(b6 * x) + a5 * exp(b5 * y) + a4 * x ^ 4 + b4 * y ^ 4 + a3 * x ^ 3 + b3 * y ^ 3 + a2 * x ^ 2 + b2 * y ^ 2 + a1 * x ^ 1 + b1 * y ^ 1 + a0 + b0 + ab * x * y + a2b * x ^ 2 * y + ab2 * x * y ^ 2+ a3b * x ^ 3 * y + ab3 * x * y ^ 3 + a3b2 * x ^ 3 * y ^ 2 + a2b3 * x ^ 2 * y ^ 3',...
'dependent',{'z'},'independent',{'x', 'y'},...
'coefficients',{'a6','b6', 'a5','b5', 'a4','b4', 'a3','b3', 'a2','b2', 'a1','b1', 'a0','b0', 'ab','a2b','ab2','a3b','ab3', 'a3b2', 'a2b3'});

exp_poly3 = fittype('a6 * exp(b6 * x) + a5 * exp(b5 * y) + a3 * x ^ 3 + b3 * y ^ 3 + a2 * x ^ 2 + b2 * y ^ 2 + a1 * x ^ 1 + b1 * y ^ 1 + a0 + b0 + ab * x * y + a2b * x ^ 2 * y + ab2 * x * y ^ 2',...
'dependent',{'z'},'independent',{'x', 'y'},...
'coefficients',{'a6','b6', 'a5','b5', 'a3','b3', 'a2','b2', 'a1','b1', 'a0','b0', 'ab','a2b','ab2',});

exp_lin = fittype('a5 * exp(b5 * x + b6 * y) + a4 * exp(b4 * x) + a3 * exp(b3 * y) + a1 * x ^ 1 + b1 * y ^ 1 + a0 + b0' ,...
'dependent',{'z'},'independent',{'x', 'y'},...
'coefficients',{'b6', 'a5','b5', 'a4','b4', 'a3','b3', 'a1','b1', 'a0','b0'});

ftype = 'exp1'

ff = fit([xx yy],zz,exp_lin)

% f = fit([x y],z,ftype)

% g = fit(x, y, 2);

% plot(f,[x y],z)
% hold on
plot(ff,[xx yy],zz)
legend('f', 'ff')