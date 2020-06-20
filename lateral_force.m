RotorPlacementAngles = [30, 90, 150, 210, 270, 330];
RotorRotationDirections = [-1, 1, -1, 1, -1, 1];
m = multirotor(RotorPlacementAngles, RotorRotationDirections);
m = m.SetRotorAngles(0, [-30, 30, -30, 30, -30, 30], 0);

RotorSpeeds = [0, 0, 0, 0, 0, 0];

flag = true;
Fx = [];
Fy = [];
Fz = [];
while flag
    m = m.UpdateState(RotorSpeeds);
    Fx = [Fx; m.Force(1)];
    Fy = [Fy; m.Force(2)];
    Fz = [Fz; m.Force(3)];
    [RotorSpeeds, flag] = NextPerm(RotorSpeeds, 0, 25000, 5000);
end

k = convhull(Fx, Fy, Fz);
trisurf(k, Fx, Fy, Fz, 'FaceColor','cyan', 'LineStyle', '-');
axis equal

figure;
Zq = [0, 1, 2, 3, 4, 5, 6, 7, 8];
xylimits = [-2.5, 2.5];
for i = 1 : 9
    subplot(3, 3, i);
    [xc, yc] = CrossSection(Fx, Fy, Fz, Zq(i));
    plot(xc, yc);
    xlabel('F_x');
    ylabel('F_y');
    title(num2str(Zq(i)));
    xlim(xylimits);
    ylim(xylimits);
    axis equal
end

function [perm, flag] = NextPerm(vec, min, max, step)
    n = length(vec);
    perm = vec;
    curr = n;
    while curr > 0
        if perm(curr) < max
            perm(curr) = perm(curr) + step;
            flag = true;
            return;
        end
        for i = curr : n
            perm(i) = min;
        end
        curr = curr - 1;
    end
    flag = false;
end

function [x, y] = CrossSection(X, Y, Z, z)
    Nq = 1e6;
    minx = min(X);
    miny = min(Y);
    maxx = max(X);
    maxy = max(Y);
    
    Xq = rand(Nq, 1) * (maxx - minx) + minx;
    Yq = rand(Nq, 1) * (maxy - miny) + miny;
    
    in1 = inhull([Xq, Yq, z*ones(Nq, 1)], [X, Y, Z]);
    x = Xq(in1);
    y = Yq(in1);
end 
        
        