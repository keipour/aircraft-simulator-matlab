classdef simulation < handle
    properties
        TotalTime = 10;      % in secs
        TimeStep = 1e-3;    % in secs
        Multirotor multirotor
    end
    
    properties(SetAccess=protected, GetAccess=public)
        CurrentTime = 0;    % in secs
        CurrentState
    end
    
    properties(SetAccess=protected, GetAccess=protected)
        InitialMultirotor multirotor
        StateHistory
    end
    
    %% Methods
    methods
        function obj = simulation(multirotor)
            obj.InitialMultirotor = multirotor;
            obj.Reset();
        end
        
        function t = GetTimeSteps(obj)
            t = 0 : obj.TimeStep : obj.TotalTime;
        end
        
        function Reset(obj)
            obj.Multirotor = multirotor(0, 1);
            obj.Multirotor.CopyFrom(obj.InitialMultirotor);
            obj.CurrentTime = 0;
            obj.StateHistory = state_collection();
            obj.StateHistory.SetCapacity(length(obj.GetTimeSteps()));
            obj.StateHistory.PushBack(obj.Multirotor.State);
        end
        
        function NextStep(obj, RotorSpeedsSquared)
            obj.Multirotor.UpdateState(RotorSpeedsSquared, obj.TimeStep);
            obj.StateHistory.PushBack(obj.Multirotor.State);
            obj.CurrentTime = obj.CurrentTime + obj.TimeStep;
        end
        
        function flag = IsLastStep(obj)
            if obj.CurrentTime + obj.TimeStep > obj.TotalTime + 1e-6
                flag = true;
            else
                flag = false;
            end
        end
        
        function traj = GetStateTrajectory(obj)
            traj = obj.StateHistory;
        end
        
        function Simulate(obj, RotorSpeedsSquared)
            obj.Reset();
            while true
                obj.NextStep(RotorSpeedsSquared);
                if obj.IsLastStep()
                    break;
                end
            end
        end
        
        function set.TotalTime(obj, value)
            obj.TotalTime = value;      % in secs
            obj.Reset();
        end

        function set.TimeStep(obj, value)
            obj.TimeStep = value;      % in secs
            obj.Reset();
        end
    end
end

