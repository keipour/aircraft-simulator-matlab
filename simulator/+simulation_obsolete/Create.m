function sim = Create(multirotor)
    sim.TotalTime = 5;      % in secs
    sim.TimeStep = 1e-3;    % in secs

    sim.InitialMultirotor = multirotor;
    sim = simulation.Reset(sim);
end
