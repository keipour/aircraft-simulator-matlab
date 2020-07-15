classdef simulation < handle
    properties
        Multirotor multirotor
        Controller controller
    end
    
    properties(Constant)
        Timer support_files.timer = support_files.timer;       % in secs
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
            obj.Timer.Reset();
            
            % Keep some state fields
            istate = obj.Multirotor.InitialState;
            obj.Multirotor.CopyFrom(obj.InitialMultirotor);
            obj.Multirotor.SetInitialState(istate.Position, istate.Velocity, istate.RPY, istate.Omega);
            
            logger.Reset();
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
        
        function NextStepPlant(obj, time)
        % Update the plant state for the next time step and advance time
            rotor_speeds_squared = last_commands.RotorSpeedsSquaredCommand.Data;
            if ~last_commands.RotorSpeedsSquaredCommand.IsInitialized()
                rotor_speeds_squared = zeros(obj.Multirotor.NumOfRotors, 1);
            end
            obj.Multirotor.UpdateState(rotor_speeds_squared, time);
            logger.Add(logger_signals.MeasuredStates, obj.Multirotor.State);
        end
        
        function NextStepControlAllocation(obj, time)
            if ~last_commands.DesiredEulerAcceleration.IsInitialized() || ...
                    ~last_commands.DesiredLinearAcceleration.IsInitialized()
                return;
            end
            lin_acc_des = last_commands.DesiredLinearAcceleration.Data;
            euler_acc_des = last_commands.DesiredEulerAcceleration.Data;
            rotor_speeds_squared = obj.Controller.ControlAcceleration(obj.Multirotor, lin_acc_des, euler_acc_des);
            last_commands.RotorSpeedsSquaredCommand.Set(rotor_speeds_squared, time);
            logger.Add(logger_signals.RotorSpeedsSquaredCommand, rotor_speeds_squared);
        end
        
        function NextStepAttitudeController(obj, time)
        % Calculate the multirotor command for a desired attitude
            if ~last_commands.DesiredRPY.IsInitialized()
                return;
            end
            rpy_des = last_commands.DesiredRPY.Data;
            euler_accel = obj.Controller.ControlAttitude(obj.Multirotor, rpy_des, time);
            last_commands.DesiredEulerAcceleration.Set(euler_accel, time);
            logger.Add(logger_signals.DesiredEulerAcceleration, euler_accel);
        end
        
        function NextStepPositionController(obj, time)
        % Calculate the multirotor command for a desired position and yaw
            if ~last_commands.DesiredPositionYaw.IsInitialized()
                return;
            end
            pos_yaw_des = last_commands.DesiredPositionYaw.Data;
            [lin_accel, rpy_des] = obj.Controller.ControlPosition(obj.Multirotor, pos_yaw_des(1 : 3), pos_yaw_des(4), time);
            last_commands.DesiredRPY.Set(rpy_des, time);
            last_commands.DesiredLinearAcceleration.Set(lin_accel, time);
            logger.Add(logger_signals.DesiredRPY, rpy_des);
            logger.Add(logger_signals.DesiredLinearAcceleration, lin_accel);
        end
        
        function NextSimulationStep(obj)
            [time, module] = obj.Timer.NextTimeStep();
            if module == obj.Timer.PlantIndex
                if last_commands.DesiredEulerAcceleration.IsInitialized() && ...
                        last_commands.DesiredLinearAcceleration.IsInitialized()
                    obj.NextStepControlAllocation(time);
                end
                obj.NextStepPlant(time);
            elseif module == obj.Timer.PosControllerIndex && last_commands.DesiredPositionYaw.IsInitialized()
                obj.NextStepPositionController(time);
            elseif module == obj.Timer.AttControllerIndex && last_commands.DesiredRPY.IsInitialized()
                obj.NextStepAttitudeController(time);
            end
        end
        
        function res = SimulateAttitudeResponse(obj, rpy_des, plot)
        % Simulate the response to a desired attitude input
            
            obj.Reset();
            last_commands.DesiredLinearAcceleration.Set(zeros(3, 1), 0);
            last_commands.DesiredRPY.Set(rpy_des, 0);
            while ~obj.Timer.IsFinished()
                obj.NextSimulationStep();
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
            last_commands.DesiredPositionYaw.Set([pos_des; yaw_des], 0);
            while ~obj.Timer.IsFinished()
                obj.NextSimulationStep();
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
