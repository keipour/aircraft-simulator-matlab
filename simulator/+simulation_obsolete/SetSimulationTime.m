function sim = SetSimulationTime(sim, value)
    sim.TotalTime = value;      % in secs
    simulation.Reset(sim);
end
