classdef logger < handle

    properties(Constant)
        Data = support_files.queue(logger_signals.Max);
    end
    
    methods(Static)
        function Reset()
            logger.Data.Reset(logger_signals.Max);
        end
        
        function Add(signal, value)
            logger.Data.Add(signal, value, simulation.Timer.CurrentTime);
        end
        
        function [data, times] = GetData(signal)
            [data, times] = logger.Data.Get(signal);
        end

        %% Functions for getting the data
        
        function [data, times] = GetMeasuredStates()
            [data, times] = logger.GetData(logger_signals.MeasuredStates);
        end
        
        function [data, times] = GetMeasuredAccelerations()
            [measured_data, times] = logger.GetMeasuredStates();
            data = cell2mat(cellfun(@(s)s.Acceleration', measured_data, 'uni', 0));
        end
        
        function [data, times] = GetMeasuredEulerRates()
            [measured_data, times] = logger.GetMeasuredStates();
            data = cell2mat(cellfun(@(s)s.EulerRate', measured_data, 'uni', 0));
        end

        function [data, times] = GetMeasuredAngularAccelerations()
            [measured_data, times] = logger.GetMeasuredStates();
            data = cell2mat(cellfun(@(s)s.AngularAcceleration', measured_data, 'uni', 0));
        end

        function [data, times] = GetMeasuredPositions()
            [measured_data, times] = logger.GetMeasuredStates();
            data = cell2mat(cellfun(@(s)s.Position', measured_data, 'uni', 0));
        end

        function [data, times] = GetMeasuredVelocities()
            [measured_data, times] = logger.GetMeasuredStates();
            data = cell2mat(cellfun(@(s)s.Velocity', measured_data, 'uni', 0));
        end

        function [data, times] = GetMeasuredRPYs()
            [measured_data, times] = logger.GetMeasuredStates();
            data = cell2mat(cellfun(@(s)s.RPY', measured_data, 'uni', 0));
        end

        function [data, times] = GetMeasuredOmegas()
            [measured_data, times] = logger.GetMeasuredStates();
            data = cell2mat(cellfun(@(s)s.Omega', measured_data, 'uni', 0));
        end

        function [data, times] = GetMeasuredForces()
            [measured_data, times] = logger.GetMeasuredStates();
            data = cell2mat(cellfun(@(s)s.Force', measured_data, 'uni', 0));
        end

        function [data, times] = GetMeasuredMoments()
            [measured_data, times] = logger.GetMeasuredStates();
            data = cell2mat(cellfun(@(s)s.Moment', measured_data, 'uni', 0));
        end
        
        function [data, times] = GetMeasuredRotorSpeeds()
            [measured_data, times] = logger.GetMeasuredStates();
            data = cell2mat(cellfun(@(s)s.RotorSpeeds', measured_data, 'uni', 0));
        end
        
        function [data, times] = GetAppliedWindForces()
            [measured_data, times] = logger.GetMeasuredStates();
            data = cell2mat(cellfun(@(s)s.WindForce', measured_data, 'uni', 0));
        end
        
        function [data, times] = GetMeasuredRotorSaturations()
            [measured_data, times] = logger.GetMeasuredStates();
            data = cell2mat(cellfun(@(s)s.RotorsSaturated', measured_data, 'uni', 0));
        end
        
        function [data, times] = GetDesiredEulerAcceleration()
            [data, times] = logger.GetData(logger_signals.DesiredEulerAcceleration);
            data = cell2mat(cellfun(@(s)s', data, 'uni', 0));
        end
        
        function [data, times] = GetDesiredLinearAcceleration()
            [data, times] = logger.GetData(logger_signals.DesiredLinearAcceleration);
            data = cell2mat(cellfun(@(s)s', data, 'uni', 0));
        end
        
        function [data, times] = GetDesiredRPY()
            [data, times] = logger.GetData(logger_signals.DesiredRPY);
            data = cell2mat(cellfun(@(s)s', data, 'uni', 0));
        end
        
        function [data, times] = GetDesiredPositionYaw()
            [data, times] = logger.GetData(logger_signals.DesiredPositionYaw);
            data = cell2mat(cellfun(@(s)s', data, 'uni', 0));
        end
        
        function [data, times] = GetRotorSpeedsSquaredCommand()
            [data, times] = logger.GetData(logger_signals.RotorSpeedsSquaredCommand);
            data = cell2mat(cellfun(@(s)s', data, 'uni', 0));
        end
        
        function [data, times] = GetMeasuredEndEffectorPositions()
            [measured_data, times] = logger.GetMeasuredStates();
            data = cell2mat(cellfun(@(s)s.EndEffectorPosition', measured_data, 'uni', 0));
        end

        function [data, times] = GetMeasuredEndEffectorVelocities()
            [measured_data, times] = logger.GetMeasuredStates();
            data = cell2mat(cellfun(@(s)s.EndEffectorVelocity', measured_data, 'uni', 0));
        end

        function [data, times] = GetCollisionStatus()
            [measured_data, times] = logger.GetMeasuredStates();
            data = cell2mat(cellfun(@(s)s.InCollision', measured_data, 'uni', 0));
        end

        function [data, times] = GetForceSensorReadings()
            [measured_data, times] = logger.GetMeasuredStates();
            data = cell2mat(cellfun(@(s)s.ForceSensor', measured_data, 'uni', 0));
        end

        function [data, times] = GetMomentSensorReadings()
            [measured_data, times] = logger.GetMeasuredStates();
            data = cell2mat(cellfun(@(s)s.MomentSensor', measured_data, 'uni', 0));
        end

        function [data, times, labels] = GetField(str)
            str = lower(str);
            
            if contains(str, 'accel') && ~contains_or(str, {'ang', 'rot'})
                [data_meas, times_meas] = logger.GetMeasuredAccelerations();
                [data_des, times_des] = logger.GetDesiredLinearAcceleration();
                if ~isempty(data_des)
                    data = {data_meas; data_des};
                    times = {times_meas, times_des};
                else
                    data = data_meas;
                    times = times_meas;
                end
                labels = {'a_x', 'a_y', 'a_z', 'Acceleration'};
                
            elseif contains_or(str, {'euler', 'rpy', 'att', 'phi'}) && ...
                    contains_or(str, {'deriv', 'dot', 'vel', 'speed', 'rate'})
                [data, times] = logger.GetMeasuredEulerRates();
                labels = {'Roll Rate', 'Pitch Rate', 'Yaw Rate', 'Attitude Rate'};

            elseif (contains_or(str, {'ang', 'rot'}) && contains(str, 'accel')) || ...
                    (contains_or(str, {'dot', 'deriv'}) && contains(str, 'omega'))
                [data_meas, times_meas] = logger.GetMeasuredAngularAccelerations();
                [data_des, times_des] = logger.GetDesiredEulerAcceleration();
                if ~isempty(data_des)
                    data = {data_meas; data_des};
                    times = {times_meas, times_des};
                else
                    data = data_meas;
                    times = times_meas;
                end
                labels = {'\alpha_x', '\alpha_y', '\alpha_z', 'Angular Acceleration'};

            elseif contains_or(str, {'euler', 'rpy', 'att', 'phi'})
                [data_meas, times_meas] = logger.GetMeasuredRPYs();
                [data_des, times_des] = logger.GetDesiredRPY();
                if ~isempty(data_des)
                    data = {data_meas; data_des};
                    times = {times_meas, times_des};
                else
                    data = data_meas;
                    times = times_meas;
                end
                labels = {'Roll', 'Pitch', 'Yaw', 'Attitude'};

            elseif contains(str, 'rpm') || (contains_or(str, {'mot', 'rotor'}) && contains_or(str, {'vel', 'rate', 'speed'}))
                [data, times] = logger.GetMeasuredRotorSpeeds();
                n_rotors = size(data, 2);
                labels = cell(1, n_rotors + 1);
                for i = 1 : n_rotors
                    labels{i} = ['Motor ' num2str(i, '%d')];
                end
                labels{n_rotors + 1} = 'Motor Velocity';

            elseif contains(str, 'omega') || (contains_or(str, {'ang', 'rot'}) && contains_or(str, {'vel', 'rate', 'speed'}))
                [data, times] = logger.GetMeasuredOmegas();
                labels = {'\omega_x', '\omega_y', '\omega_z', 'Angular Velocity'};

            elseif contains_or(str, {'air', 'wind'})
                [data, times] = logger.GetAppliedWindForces();
                labels = {'F_x', 'F_y', 'F_z', 'Wind Force'};

            elseif contains_and(str, {'forc', 'sens'})
                [data, times] = logger.GetForceSensorReadings();
                labels = {'F_x', 'F_y', 'F_z', 'Force Sensor'};

            elseif contains(str, 'forc')
                [data, times] = logger.GetMeasuredForces();
                labels = {'F_x', 'F_y', 'F_z', 'Generated Force'};

            elseif contains_or(str, {'momen', 'torq'}) && contains(str, 'sens')
                [data, times] = logger.GetMomentSensorReadings();
                labels = {'M_x', 'M_y', 'M_z', 'Moment Sensor'};

            elseif contains_or(str, {'momen', 'torq'})
                [data, times] = logger.GetMeasuredMoments();
                labels = {'M_x', 'M_y', 'M_z', 'Generated Moment'};

            elseif contains_or(str, {'vel', 'speed'}) && contains(str, 'eff')
                [data, times] = logger.GetMeasuredEndEffectorVelocities();
                labels = {'V_xe', 'V_ye', 'V_ze', 'End Effector Velocity'};

            elseif contains_or(str, {'vel', 'speed'})
                [data, times] = logger.GetMeasuredVelocities();
                labels = {'V_x', 'V_y', 'V_z', 'Velocity'};
            
            elseif contains(str, 'eff')
                [data, times] = logger.GetMeasuredEndEffectorPositions();
                labels = {'x_e', 'y_e', 'z_e', 'End Effector Position'};

            elseif contains(str, 'pos')
                [data_meas, times_meas] = logger.GetMeasuredPositions();
                [data_des, times_des] = logger.GetDesiredPositionYaw();
                if ~isempty(data_des)
                    data = {data_meas; data_des(:, 1:3)};
                    times = {times_meas, times_des};
                else
                    data = data_meas;
                    times = times_meas;
                end
                labels = {'x', 'y', 'z', 'Position'};

            elseif contains(str, 'sat')
                [data, times] = logger.GetMeasuredRotorSaturations();
                labels = {'Motor Saturations', 'Motor Saturations'};

            elseif contains_or(str, {'contact', 'colli'})
                [data, times] = logger.GetContactStatus();
                labels = {'Contact Status', 'Contact Status'};

            else
                error('Input string not recognized.');
            end
        end
        
    end
end

%% Helper functions
function res = contains_or(str, patterns)
    res = false;
    if ~iscell(patterns)
        patterns = {patterns};
    end
    for i = 1 : length(patterns)
        res = res || contains(str, patterns{i});
    end
end

function res = contains_and(str, patterns)
    res = true;
    if ~iscell(patterns)
        patterns = {patterns};
    end
    for i = 1 : length(patterns)
        res = res && contains(str, patterns{i});
    end
end
