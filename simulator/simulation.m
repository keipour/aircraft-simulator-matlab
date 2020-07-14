classdef simulation < handle
    properties
        Multirotor multirotor
        Controller controller
    end
    
    properties(Constant)
        Timer = support_files.timer;       % in secs
    end
    
    properties(SetAccess=protected, GetAccess=protected)
        InitialMultirotor multirotor
    end
    
    %% Methods
    methods
        function obj = simulation(multrotor, controller)
            obj.InitialMultirotor = multrotor;
            obj.Controller = controller;
            obj.Multirotor = multirotor(0, 1);
            obj.Reset();
        end

        function Reset(obj)
            obj.Controller.Reset();
            
            % Keep some state fields
            istate = obj.Multirotor.InitialState;
            obj.Multirotor.CopyFrom(obj.InitialMultirotor);
            obj.Multirotor.SetInitialState(istate.Position, istate.Velocity, istate.RPY, istate.Omega);
            
            logger.Reset();
            logger.Add(logger_signals.MeasuredStates, obj.Multirotor.State);
        end
        
        function SetTotalTime(obj, value)
            obj.Timer.TotalTime = value;      % in secs
            obj.Reset();
        end

        function flag = IsLastStep(obj)
            if obj.Timer.CurrentTime + 1 / obj.Timer.PlantRate > obj.Timer.TotalTime + 1e-6
                flag = true;
            else
                flag = false;
            end
        end
        
        function NextStepPlant(obj, rotor_speeds_squared)
        % Update the plant state for the next time step and advance time
        
            obj.Timer.CurrentTime = obj.Timer.CurrentTime + 1 / obj.Timer.PlantRate;
            obj.Multirotor.UpdateState(rotor_speeds_squared, 1 / obj.Timer.PlantRate);
            logger.Add(logger_signals.MeasuredStates, obj.Multirotor.State);
        end
        
        function rotor_speeds_squared = NextAttitudeCommand(obj, rpy_des, lin_accel)
        % Calculate the multirotor command for a desired attitude
        
            rotor_speeds_squared = obj.Controller.ControlAttitude(obj.Multirotor, rpy_des, lin_accel, 1 / obj.Timer.PlantRate);
        end
        
        function rotor_speeds_squared = NextPositionCommand(obj, pos_des, yaw_des)
        % Calculate the multirotor command for a desired position and yaw

            rotor_speeds_squared = obj.Controller.ControlPosition(obj.Multirotor, pos_des, yaw_des, 1 / obj.Timer.PlantRate);
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
            [rpys, rpy_times] = logger.GetMeasuredRPYs();
            res = analysis.AnalyzeAndOutputResponse(rpy_times, rpys, ...
                rpy_des, signal_names, plot);
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
            [pos_res, pos_times] = logger.GetMeasuredPositions();
            [rpy_res, ~] = logger.GetMeasuredRPYs();
            res = analysis.AnalyzeAndOutputResponse(pos_times, ...
                [pos_res, rpy_res(:, 3)], [pos_des; yaw_des], signal_names, plot);
        end
        
    end
end
