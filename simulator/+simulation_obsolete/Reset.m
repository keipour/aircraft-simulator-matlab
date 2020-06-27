function obj = Reset(obj)
    obj.Multirotor = obj.InitialMultirotor;
    obj.CurrentTime = 0;
    obj.StepIndex = 1;
    obj.StateHistory = cell(length(simulation.GetTimeSteps(obj)), 1);
    obj.StateHistory{1} = obj.Multirotor.State;
end
