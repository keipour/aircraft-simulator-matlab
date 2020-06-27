function flag = IsLastStep(obj)
    if obj.CurrentTime + obj.TimeStep > obj.TotalTime + 1e-6
        flag = true;
    else
        flag = false;
    end
end
