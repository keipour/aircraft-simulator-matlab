classdef simulation < handle
    properties
        Multirotor multirotor
        Controller controller
        Environment environment
        TrajectoryController trajectory_controller
    end
    
    properties(Constant)
        Timer support_files.timer = support_files.timer;       % in secs
    end
    
    properties(SetAccess=protected, GetAccess=protected)
        InitialMultirotor multirotor
    end
    
    %% Methods
    methods
        function obj = simulation(multrotor, contrller, envronment)
            if nargin < 2
                contrller = controller(multrotor);
            end
            if nargin < 3
                envronment = environment();
            end
            
            obj.InitialMultirotor = multrotor;
            obj.Controller = contrller;
            obj.Multirotor = multirotor(0, 1);
            obj.Environment = envronment;
            obj.TrajectoryController = trajectory_controller;
            obj.Reset();
        end

        function Reset(obj)
            obj.Controller.Reset();
            obj.Timer.Reset();
            obj.TrajectoryController = trajectory_controller;

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
            obj.UpdateAllStates(rotor_speeds_squared, time);
            logger.Add(logger_signals.MeasuredStates, struct(obj.Multirotor.State));
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
        
        function NextStepTrajectoryController(obj, time)
        % Calculate the multirotor command for a desired trajectpry
            
            pose = [obj.Multirotor.State.Position; obj.Multirotor.State.RPY(3)];
            next_wp = obj.TrajectoryController.CalcLookaheadPoint(pose);
            if time >= 0
                logger.Add(logger_signals.DesiredPositionYaw, next_wp);
            else
                time = 0;
            end
            last_commands.DesiredPositionYaw.Set(next_wp, time);
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
            elseif module == obj.Timer.TrajControllerIndex && obj.TrajectoryController.IsInitialized()
                obj.NextStepTrajectoryController(time);
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
        
        function res = SimulateTrajectory(obj, traj_des, radius)
        % Simulate the response to a desired attitude input
            
            obj.Reset();
            obj.TrajectoryController.SetWaypoints(traj_des, radius);

            obj.NextStepTrajectoryController(-1);
            while ~obj.Timer.IsFinished()
                obj.NextSimulationStep();
            end
        end
    end
    
    %% Private methods
    
    methods (Access = private)

        function UpdateAllStates(obj, rotor_speeds_squared, time)
            
            % Save the last time we updated the state
            persistent last_time;
            if isempty(last_time)
                last_time = 0;
            end
            
            % Calculate the delta-t from the last time we updated the state
            dt = time - last_time;

            % Save the current time for the next loop iteration
            last_time = time;
            
            % Calculate the wind_force applied to the multirotor
            air_velocity = physics.GetAirVelocity(obj.Multirotor.State.Velocity, obj.Environment.AverageWind);
            eff_wind_area = obj.Multirotor.WindModel.CalcualteEffectiveArea(air_velocity, obj.Multirotor.State.RPY);
            wind_force = physics.GetWindForce(air_velocity, eff_wind_area);
            
            % Calculate the next state of the robot if there are no collisions
            [wrench] = obj.Multirotor.CalcGeneratedWrench(rotor_speeds_squared);
            new_state = obj.Multirotor.CalcNextState(wrench, ...
                zeros(6, 1), wind_force, rotor_speeds_squared, dt, false, zeros(3, 1));
            
            % Check for collistion in the new potential state
            cm = obj.Multirotor.GetTransformedCollisionModel(new_state.Position, deg2rad(new_state.RPY));
            collision = physics.CheckAllCollisions(cm, obj.Environment.CollisionModels);
            
            if collision == true
                
                wall_normal = [-1; 0; 0];
                [vel_mat, force_mat] = get_free_contact_matrices(wall_normal);

                force_sensor = zeros(3, 1);
                if obj.Multirotor.HasEndEffector()

                    % Calculate the force sensor reading
                    collision_force = -dot(wrench(4:6), wall_normal);
                    if collision_force < 0
                        collision_force = 0;
                    end

                    collision_force_vec = collision_force * wall_normal;
                    R_SI = obj.Multirotor.GetRotationMatrix()' * obj.Multirotor.EndEffector.R_BE;
                    force_sensor = R_SI' * collision_force_vec;
                end
                
                torque_sensor = zeros(3, 1);
                new_state = obj.Multirotor.CalcNextState(wrench, [torque_sensor; force_sensor],...
                    wind_force, rotor_speeds_squared, dt, true, wall_normal);
            end
            
            % Save the calculated next state
            obj.Multirotor.UpdateState(new_state);
        end
        
    end
end

%% Helper functions

function [vel_mat, force_mat] = get_free_contact_matrices(normal)
    base_vec = [0; 0; 1];
    orth_vec1 = [0; 1; 0];
    orth_vec2 = [1; 0; 0];
    
    R = vrrotvec2mat(vrrotvec([cos(pi/4); 0; cos(pi/4)], base_vec));
    o1 = R' * orth_vec1;
    o2 = R' * orth_vec2;
    
    
    
    Rot = blkdiag(R, R);

    base_vel_mat = diag([0, 1, 1, 1, 1, 1]);
    base_vel_mat = diag([1, 1, 0, 1, 1, 1]);
    vel_mat = Rot * base_vel_mat;    

    base_force_mat = eye(6) - base_vel_mat;
    force_mat = Rot * base_force_mat;
end