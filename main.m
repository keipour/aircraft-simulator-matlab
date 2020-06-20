RotorPlacementAngles = [30, 90, 150, 210, 270, 330];
RotorRotationDirections = [-1, 1, -1, 1, -1, 1];
RotorDihedralAngle = 0;
RotorSidewardAngle = [-30, 30, -30, 30, -30, 30];
RotorInwardAngle = 0;

m = multirotor(RotorPlacementAngles, RotorRotationDirections);
m = m.SetRotorAngles(RotorInwardAngle, RotorSidewardAngle, RotorDihedralAngle);
m.I = calc_inertia(m.Rotors(1).ArmLength, RotorPlacementAngles, RotorDihedralAngle, ...
    m.Mass - 6 * (m.Rotors(1).ArmMass + m.Rotors(1).MotorMass), ...
    m.Rotors(1).ArmMass, m.Rotors(1).MotorMass, 0.15);

RotorSpeedsSquared = [55000, 55000, 55000, 55000, 55000, 55000];
tic
X = {};
for i = 1 : 100
    m = m.UpdateState(RotorSpeedsSquared, 0.01);
    X{i, 1} = m.GetState();
end
toc

Pos = cell2mat(cellfun(@(s)s.Position', X, 'uni', 0));

subplot(3, 1, 1);
plot((1:100) * 0.01, Pos(:, 3))