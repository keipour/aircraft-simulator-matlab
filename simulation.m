classdef simulation
    %SIMULATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        TotalTime = 5;      % in secs
        TimeStep = 1e-3;    % in secs
        Multirotor multirotor
    end
    
    properties(SetAccess=protected, GetAccess=public)
        CurrentTime = 0;    % in secs
        StepIndex = 1;
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
            obj = obj.Reset();
        end
        
        function t = GetTimeSteps(obj)
            t = 0 : obj.TimeStep : obj.TotalTime;
        end
        
        function obj = Reset(obj)
            obj.Multirotor = obj.InitialMultirotor;
            obj.CurrentTime = 0;
            obj.StepIndex = 1;
            obj.StateHistory = cell(length(obj.GetTimeSteps()), 1);
            %obj.StateHistory{1, 1} = obj.Multirotor.State;
        end
        
        function obj = NextStep(obj, RotorSpeedsSquared)
            obj.StepIndex = obj.StepIndex + 1;
            obj.Multirotor = obj.Multirotor.UpdateState(RotorSpeedsSquared, obj.TimeStep);
            %obj.StateHistory{obj.StepIndex, 1} = obj.Multirotor.State;
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
    end
end

