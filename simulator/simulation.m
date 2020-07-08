classdef simulation < handle
    properties
        Multirotor multirotor
        Controller controller
    end
    
    properties(Constant)
        Timer = simulation_support.timer;       % in secs
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
            t = 0 : obj.Timer.TimeStep : obj.Timer.TotalTime;
        end
        
        function Reset(obj)
            obj.Controller.Reset();
            
            % Keep some state fields
            istate = obj.Multirotor.InitialState;
            obj.Multirotor.CopyFrom(obj.InitialMultirotor);
            obj.Multirotor.SetInitialState(istate.Position, istate.Velocity, istate.RPY, istate.Omega);
            
            obj.Timer.CurrentTime = 0;
            obj.StateHistory = state_collection(obj.Multirotor.NumOfRotors);
            obj.StateHistory.SetCapacity(length(obj.GetTimeSteps()));
            obj.StateHistory.PushBack(obj.Multirotor.State);
        end
        
        function SetTotalTime(obj, value)
            obj.Timer.TotalTime = value;      % in secs
            obj.Reset();
        end

        function SetTimeStep(obj, value)
            obj.Timer.TimeStep = value;      % in secs
            obj.Reset();
        end
        
        function flag = IsLastStep(obj)
            if obj.Timer.CurrentTime + obj.Timer.TimeStep > obj.Timer.TotalTime + 1e-6
                flag = true;
            else
                flag = false;
            end
        end
        
        function traj = GetStateTrajectory(obj)
            traj = obj.StateHistory;
        end
        
        function NextStepPlant(obj, rotor_speeds_squared)
        % Update the plant state for the next time step and advance time
        
            obj.Multirotor.UpdateState(rotor_speeds_squared, obj.Timer.TimeStep);
            obj.StateHistory.PushBack(obj.Multirotor.State);
            obj.Timer.CurrentTime = obj.Timer.CurrentTime + obj.Timer.TimeStep;
        end
        
        function rotor_speeds_squared = NextAttitudeCommand(obj, rpy_des, lin_accel)
        % Calculate the multirotor command for a desired attitude
        
            rotor_speeds_squared = obj.Controller.ControlAttitude(obj.Multirotor, rpy_des, lin_accel, obj.Timer.TimeStep);
        end
        
        function rotor_speeds_squared = NextPositionCommand(obj, pos_des, yaw_des)
        % Calculate the multirotor command for a desired position and yaw

            rotor_speeds_squared = obj.Controller.ControlPosition(obj.Multirotor, pos_des, yaw_des, obj.Timer.TimeStep);
        end
        
        function res = SimulateAttitudeResponse(obj, rpy_des, plot)
        % Simulate the response to a desired attitude input
            
            obj.Reset();
            lin_accel = zeros(3, 1);
            while true
                u = obj.NextAttitudeCommand(rpy_des, lin_accel);
                obj.NextStepPlant(u);
                if obj.IsLastStep()
                    break;
                end
            end
            
            % Analysis of the response
            signal_names = {'Roll', 'Pitch', 'Yaw'};
            res = analysis.AnalyzeAndOutputResponse(obj.GetTimeSteps(), ...
                obj.StateHistory.GetRPYs(), rpy_des, signal_names, plot);
        end
        
        function res = SimulatePositionResponse(obj, pos_des, yaw_des, plot)
        % Simulate the response to a desired attitude input
            
            obj.Reset();
            while true
                u = obj.NextPositionCommand(pos_des, yaw_des);
                obj.NextStepPlant(u);
                if obj.IsLastStep()
                    break;
                end
            end
            
            % Analysis of the response
            signal_names = {'X', 'Y', 'Z', 'Yaw'};
            pos_res = obj.StateHistory.GetPositions();
            rpy_res = obj.StateHistory.GetRPYs();
            res = analysis.AnalyzeAndOutputResponse(obj.GetTimeSteps(), ...
                [pos_res, rpy_res(:, 3)], [pos_des; yaw_des], signal_names, plot);
        end
        
    end
end
