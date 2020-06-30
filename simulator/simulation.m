classdef simulation < handle
    properties
        TotalTime = 5;      % in secs
        TimeStep = 1e-3;    % in secs
        Multirotor multirotor
        Controller controller
    end
    
    properties(SetAccess=protected, GetAccess=public)
        CurrentTime = 0;    % in secs
        CurrentState
    end
    
    properties(SetAccess=protected, GetAccess=protected)
        InitialMultirotor multirotor
        StateHistory state_collection
    end
    
    %% Methods
    methods
        function obj = simulation(multrotor, controller)
            obj.InitialMultirotor = multrotor;
            obj.Controller = controller;
            obj.Multirotor = multirotor(0, 1);
            obj.Reset();
        end
        
        function t = GetTimeSteps(obj)
            t = 0 : obj.TimeStep : obj.TotalTime;
        end
        
        function Reset(obj)
            obj.Controller.Reset();
            
            % Keep some state fields
            istate = obj.Multirotor.InitialState;
            obj.Multirotor.CopyFrom(obj.InitialMultirotor);
            obj.Multirotor.SetInitialState(istate.Position, istate.Velocity, istate.RPY, istate.Omega);
            
            obj.CurrentTime = 0;
            obj.StateHistory = state_collection(obj.Multirotor.NumOfRotors);
            obj.StateHistory.SetCapacity(length(obj.GetTimeSteps()));
            obj.StateHistory.PushBack(obj.Multirotor.State);
        end
        
        function NextStepPlant(obj, rotor_speeds_squared)
            obj.Multirotor.UpdateState(rotor_speeds_squared, obj.TimeStep);
            obj.StateHistory.PushBack(obj.Multirotor.State);
            obj.CurrentTime = obj.CurrentTime + obj.TimeStep;
        end
        
        function rotor_speeds_squared = ControlAttitude(obj, rpy)
            rotor_speeds_squared = obj.Controller.ControlAttitude(obj.Multirotor, rpy, obj.TimeStep);
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
        
        function res = SimulateAttitudeResponse(obj, rpy_des, plot)
            % Simulate the response
            obj.Reset();
            while true
                u = obj.ControlAttitude(rpy_des);
                obj.NextStepPlant(u);
                if obj.IsLastStep()
                    break;
                end
            end
            
            % Analysis of the response
            rpys = obj.StateHistory.GetRPYs();
            signal_names = {'Roll', 'Pitch', 'Yaw'};
            res = cell(3, 1);
            for i = 1 : 3
                res{i} = analysis.AnalyzeResponse(obj.GetTimeSteps(), rpys(:, i), rpy_des(i), signal_names{i});

                % Plot the analysis if asked
                if plot == true
                    subplot(3, 1, i);
                    graphics.PlotAnalysis(res{i});
                end
                
                % Print the analysis results
                graphics.PrintAnalysis(res{i});
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

