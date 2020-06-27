function obj = NextStep(obj, RotorSpeedsSquared)
    obj.StepIndex = obj.StepIndex + 1;
    obj.Multirotor = multirotor.UpdateState(obj.Multirotor, RotorSpeedsSquared, obj.TimeStep);
    %obj.StateHistory{obj.StepIndex} = obj.Multirotor.State;
    obj.CurrentTime = obj.CurrentTime + obj.TimeStep;
end
