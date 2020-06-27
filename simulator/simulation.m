classdef simulation < handle
    %SIMULATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        TotalTime = 5;      % in secs
        TimeStep = 1e-3;    % in secs
        Multirotor
    end
    
    properties(SetAccess=protected, GetAccess=public)
        CurrentTime = 0;    % in secs
        CurrentState
    end
    
    properties(SetAccess=protected, GetAccess=protected)
        InitialMultirotor
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
            obj.Multirotor = obj.InitialMultirotor;
            obj.CurrentTime = 0;
            obj.StateHistory = state_collection();
            obj.StateHistory.SetCapacity(length(obj.GetTimeSteps()));
            obj.StateHistory.PushBack(obj.Multirotor.State);
        end
        
        function NextStep(obj, RotorSpeedsSquared)
            obj.Multirotor = multirotor.UpdateState(obj.Multirotor, RotorSpeedsSquared, obj.TimeStep);
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
            traj = obj.StateHistory.States;
        end
    end
end

