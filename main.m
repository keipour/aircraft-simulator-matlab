RotorPlacementAngles = [30, 90, 150, 210, 270, 330];
RotorRotationDirections = [-1, 1, -1, 1, -1, 1];
RotorDihedralAngle = 0;
RotorSidewardAngle = [-30, 30, -30, 30, -30, 30];
RotorInwardAngle = 0;

m = multirotor(RotorPlacementAngles, RotorRotationDirections);
m = m.SetRotorAngles(RotorInwardAngle, RotorSidewardAngle, RotorDihedralAngle);

RotorSpeedsSquared = [55000, 55000, 55000, 55000, 55000, 55000];
tic
sim = simulation(m);
X = cell(length(sim.GetTimeSteps()), 1);
X{1} = sim.Multirotor.State;
i = 1;
while true
    sim = sim.NextStep(RotorSpeedsSquared);
    i = i + 1;
    X{i} = sim.Multirotor.State;
    if sim.IsLastStep()
        break;
    end
end
states = state_collection();
states.SetStates(X);
toc

Pos = cell2mat(cellfun(@(s)s.Position', X, 'uni', 0));

subplot(3, 1, 1);
plot(sim.GetTimeSteps(), Pos(:, 3))