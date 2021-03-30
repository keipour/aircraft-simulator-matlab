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
        WaitBar
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
            if isa(multrotor, 'vtol')
                obj.Multirotor = vtol(0, 1);
            else
                obj.Multirotor = multirotor(0, 1);
            end
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
            [contact_status, contact_normal, contact_wrench, rot_ic] = ...
                obj.UpdateAllStates(rotor_speeds_squared, time);
            if contact_status == true
                last_commands.ContactNormal.Set(contact_normal, time);
                last_commands.ContactForce.Set(rot_ic' * contact_wrench(4 : 6), time);
            end
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
            euler_accel = obj.Controller.ControlAttitude(obj.Multirotor, rpy_des, [], [], time);
            last_commands.DesiredEulerAcceleration.Set(euler_accel, time);
            logger.Add(logger_signals.DesiredEulerAcceleration, euler_accel);
        end
        
        function NextStepMotorController(obj, time)
        % Change the motor angles for a VTOL
        % TODO: Add a proper motor controller
            if ~last_commands.DesiredWaypoint.IsInitialized()
                return;
            end
            waypoint_des = last_commands.DesiredWaypoint.Data;
            if any(isnan(waypoint_des.ServoAngles))
                return;
            end
            if length(waypoint_des.ServoAngles) ~= obj.Multirotor.NumOfServos
                error('The number of servos in the waypoint is different than the number of robot servos.');
                return;
            end
            persistent servoangles last_time
            if isempty(servoangles) || isempty(last_time)
                servoangles = zeros(obj.Multirotor.NumOfServos, 1);
                for i = 1 : obj.Multirotor.NumOfServos
                    servoangles(i) = obj.Multirotor.Servos{i}.CurrentAngle;
                end
                last_time = time;
            end
            
            dt = time - last_time;
            servo_maximum_rates = zeros(obj.Multirotor.NumOfServos, 1); % deg/s
            for i = 1 : obj.Multirotor.NumOfServos
                servo_maximum_rates(i) = obj.Multirotor.Servos{i}.MaxRate;
            end
            max_change = servo_maximum_rates * dt;
            
            angleerr = servoangles - waypoint_des.ServoAngles';
            cond = abs(angleerr) <= max_change;
            servoangles(cond) = waypoint_des.ServoAngles(cond)';
            cond = angleerr > max_change;
            servoangles(cond) = servoangles(cond) - max_change(cond);
            cond = angleerr < -max_change;
            servoangles(cond) = servoangles(cond) + max_change(cond);
            
            obj.Multirotor.ChangeServoAngles(servoangles);
            
            last_time = time;
        end
        
        function NextStepPositionController(obj, time)
        % Calculate the multirotor command for a desired position and yaw
            if ~last_commands.DesiredWaypoint.IsInitialized()
                return;
            end
            waypoint_des = last_commands.DesiredWaypoint.Data;
            
            [lin_accel, rpy_des] = obj.Controller.ControlPosition(obj.Multirotor, ...
                waypoint_des.Position, waypoint_des.RPY(3), [], [], time);

            last_commands.DesiredRPY.Set(rpy_des, time);
            last_commands.DesiredLinearAcceleration.Set(lin_accel, time);
            logger.Add(logger_signals.DesiredRPY, rpy_des);
            logger.Add(logger_signals.DesiredLinearAcceleration, lin_accel);
        end
        
        function NextStepHMFController(obj, time)
        % Calculate the multirotor command for a desired motion and force
            
            if ~last_commands.DesiredWaypoint.IsInitialized() ...
                || ~last_commands.ContactNormal.IsInitialized()
                return;
            end
            
            waypoint_des = last_commands.DesiredWaypoint.Data;
            contact_normal = last_commands.ContactNormal.Data;
            
            vel_mat = diag([0, 1, 1]);
            force_constraint = [-1; 0; 0];
            [lin_accel, rpy_des] = obj.Controller.ControlMotionAndForce(obj.Multirotor, ...
                waypoint_des.Force, waypoint_des.Position, waypoint_des.RPY(3), [], [], contact_normal, ...
                vel_mat, force_constraint, time);

            last_commands.DesiredRPY.Set(rpy_des, time);
            last_commands.DesiredLinearAcceleration.Set(lin_accel, time);
            logger.Add(logger_signals.DesiredRPY, rpy_des);
            logger.Add(logger_signals.DesiredLinearAcceleration, lin_accel);
        end
        
        function NextStepTrajectoryController(obj, time)
        % Calculate the multirotor command for a desired trajectpry

            contact_force = [];
            if last_commands.ContactForce.IsInitialized()
                contact_force = last_commands.ContactForce.Data;
            end

            next_wp = obj.TrajectoryController.CalcLookaheadPoint(obj.Multirotor.State.Position, ...
                obj.Multirotor.State.RPY(3), contact_force, obj.Multirotor.State.InCollision);
            if time >= 0
                logger.Add(logger_signals.DesiredPositionYaw, [next_wp.Position; next_wp.RPY(3)]);
            else
                time = 0;
            end
            last_commands.DesiredWaypoint.Set(next_wp, time);
        end
        
        function success = NextSimulationStep(obj)
            [time, module] = obj.Timer.NextTimeStep();
            if ~update_waitbar(obj.WaitBar, time / obj.Timer.TotalTime)
                success = false;
                return;
            end
            success = true;
            if module == obj.Timer.PlantIndex
                if last_commands.DesiredEulerAcceleration.IsInitialized() && ...
                        last_commands.DesiredLinearAcceleration.IsInitialized()
                    obj.NextStepControlAllocation(time);
                end
                obj.NextStepPlant(time);
            elseif module == obj.Timer.PosControllerIndex && last_commands.DesiredWaypoint.IsInitialized()
                if obj.Multirotor.State.InCollision
                    obj.NextStepHMFController(time);
                else
                    obj.NextStepPositionController(time);
                end
            elseif module == obj.Timer.AttControllerIndex && last_commands.DesiredRPY.IsInitialized()
                obj.NextStepMotorController(time); % vtol
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
            obj.WaitBar = start_waitbar(obj.WaitBar, 'Simulating the attitude response...');
            stop = false;
            while ~obj.Timer.IsFinished() && ~stop
                stop = ~obj.NextSimulationStep();
            end
            close_waitbar(obj.WaitBar);
            
            % Analysis of the response
            signal_names = {'Roll', 'Pitch', 'Yaw'};
            [rpys, rpy_times] = logger.GetMeasuredRPYs();
            res = analysis.AnalyzeAndOutputResponse(rpy_times, rpys, ...
                rpy_des, signal_names, plot);
        end
        
        function res = SimulatePositionResponse(obj, pos_des, yaw_des, plot)
        % Simulate the response to a desired attitude input
            
            obj.Reset();
            last_commands.DesiredWaypoint.Set(support_files.waypoint([pos_des; yaw_des]), 0);
            obj.WaitBar = start_waitbar(obj.WaitBar, 'Simulating the position response...');
            stop = false;
            while ~obj.Timer.IsFinished() && ~stop
                stop = ~obj.NextSimulationStep();
            end
            close_waitbar(obj.WaitBar);
            
            % Analysis of the response
            signal_names = {'X', 'Y', 'Z', 'Yaw'};
            [pos_res, pos_times] = logger.GetMeasuredPositions();
            [rpy_res, ~] = logger.GetMeasuredRPYs();
            res = analysis.AnalyzeAndOutputResponse(pos_times, ...
                [pos_res, rpy_res(:, 3)], [pos_des; yaw_des], signal_names, plot);
        end
        
        function SimulateTrajectory(obj, traj_des, pos_thresh, rpy_thresh, force_thresh)
        % Simulate the response to a desired attitude input
            
            if nargin < 4
                rpy_thresh = [];
            end
            if nargin < 5
                force_thresh = [];
            end
            if length(rpy_thresh) == 1
                rpy_thresh = [inf; inf; rpy_thresh];
            end
            if isempty(rpy_thresh)
                rpy_thresh = inf(3, 1);
            end
            if isempty(force_thresh)
                force_thresh = inf(3, 1);
            end
        
            obj.Reset();
            obj.TrajectoryController.SetWaypoints(traj_des, pos_thresh, rpy_thresh, force_thresh);

            obj.NextStepTrajectoryController(-1);
            obj.WaitBar = start_waitbar(obj.WaitBar, 'Simulating the trajectory...');
            stop = false;
            while ~obj.Timer.IsFinished() && ~stop
                stop = ~obj.NextSimulationStep();
            end
            close_waitbar(obj.WaitBar);
        end
    end
    
    %% Private methods
    
    methods (Access = private)

        function [contact_status, contact_normal, contact_wrench, rot_ic] = ...
                UpdateAllStates(obj, rotor_speeds_squared, time)
            
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
            wrench = obj.Multirotor.CalcGeneratedWrench(rotor_speeds_squared);
            new_state = obj.Multirotor.CalcNextState(wrench, ...
                zeros(6, 1), wind_force, rotor_speeds_squared, dt, false, zeros(3, 1));
            
            % Check for collistion in the new potential state
            cm = obj.Multirotor.GetTransformedCollisionModel(new_state.Position, deg2rad(new_state.RPY));
            [~, col_ind] = physics.CheckAllCollisions(cm, obj.Environment.CollisionModels);
            
            contact_normal = [1; 0; 0];
            contact_wrench = zeros(6, 1);
            rot_ic = eye(3);
            if col_ind > 0
                
                contact_normal = obj.Environment.Objects{col_ind}.Normal;
                contact_friction = obj.Environment.Objects{col_ind}.Friction;
                %contact_normal = [-1; 0; 0];
                %contact_normal = [-cosd(30); 0; -sind(30)];

                ft_sensor = zeros(6, 1);
                if obj.Multirotor.HasEndEffector()

                    % Calculate the normal wrench resulting from the contact
                    [normal_contact_wrench, rot_ic] = physics.ApplyContactConstraints...
                        (wrench, contact_normal, diag([0, 0, 0, 1, 0, 0]), ...
                        [0; 0; 0; -1; 0; 0], [obj.Multirotor.GetRotationMatrix()'; eye(3)]);
                    normal_contact_wrench = -normal_contact_wrench;
                    
                    friction_force = physics.ApplyContactFriction(wrench(4:6), ...
                        new_state.EndEffectorVelocity, contact_normal, contact_friction, eye(3));

                    % The total wrench resulting from the contact
                    contact_wrench = normal_contact_wrench + [zeros(3, 1); friction_force];
                    
                    % Calculate the rotation from inertial to the sensor (end effector) frame
                    R_SI = obj.Multirotor.GetRotationMatrix()' * obj.Multirotor.EndEffector.R_BE;
                    
                    % Create the block diagonal rotation
                    % Note: For some weird reason, in R2019b this method of creating block 
                    % diagonals is faster than [a, zero(3); zero(3), b] and much faster 
                    % than using blkdiag function
                    blk_rot = eye(6);
                    blk_rot(1:3, 1:3) = obj.Multirotor.EndEffector.R_BE';
                    blk_rot(4:6, 4:6) = R_SI';
                    
                    % Rotate the contact wrench to the force/torque sensor frame
                    ft_sensor = blk_rot * contact_wrench;
                end
                
                new_state = obj.Multirotor.CalcNextState(wrench, ft_sensor,...
                    wind_force, rotor_speeds_squared, dt, true, contact_normal);
            end
            
            contact_status = col_ind > 0;
            
            % Save the calculated next state
            obj.Multirotor.UpdateState(new_state);
        end
        
    end
end

%% Helper functions

function h = start_waitbar(h, titletext)
    if options.SS_ShowWaitbar
        close_waitbar(h);
        h = waitbar(0, '0%%', 'Name', titletext);
    end
end

function flag = update_waitbar(h, ratio)
    flag = true;
    if options.SS_ShowWaitbar == false
        return;
    end
    
    perc = floor(ratio * 100);
    persistent last_perc
    if isempty(last_perc) || last_perc ~= perc
        last_perc = perc;
        try
            waitbar(ratio, h, sprintf('%0.0f%%', perc));
        catch
            flag = false;
        end
    end
end

function close_waitbar(h)
    try
        delete(h);
    catch
    end
end
