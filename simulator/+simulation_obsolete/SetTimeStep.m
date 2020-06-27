function sim = SetTimeStep(sim, value)
    sim.TimeStep = value;      % in secs
    simulation.Reset(sim);
end
