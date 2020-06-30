classdef state_collection < handle
    
    properties(SetAccess=protected, GetAccess=public)
        States
        Size
        Capacity
        NumOfRotors
    end
    
    methods
        function obj = state_collection(n_rotors)
            obj.NumOfRotors = n_rotors;
            obj.Reset();
        end
        
        function SetCapacity(obj, value)
            if value == 0
                obj.States = {};
                obj.Size = 0;
            elseif obj.Size >= value
                obj.States = obj.States{1:value, 1};
                obj.Size = value;
            elseif obj.Capacity >= value
                obj.States = obj.States{1:value, 1};
            else 
                obj.States{value, 1} = state.Create(obj.NumOfRotors);
            end
            obj.Capacity = value;
        end
        
        function flag = IsEmpty(obj)
            flag = obj.Size > 0;
        end
        
        function Reset(obj)
            obj.SetCapacity(0);
        end
        
        function PushBack(obj, new_state)
            obj.Size = obj.Size + 1;
            obj.States{obj.Size, 1} = new_state;
            if obj.Capacity < obj.Size
                obj.Capacity = obj.Size;
            end
        end
        
        function SetStates(obj, states)
            obj.States = states;
            obj.Size = length(states);
            obj.Capacity = length(states);
        end
        
        function res = GetAccelerations(obj)
            res = cell2mat(cellfun(@(s)s.Acceleration', obj.States, 'uni', 0));
        end

        function res = GetEulerDerivatives(obj)
            res = cell2mat(cellfun(@(s)s.EulerDerivative', obj.States, 'uni', 0));
        end

        function res = GetAngularAccelerations(obj)
            res = cell2mat(cellfun(@(s)s.AngularAcceleration', obj.States, 'uni', 0));
        end

        function res = GetPositions(obj)
            res = cell2mat(cellfun(@(s)s.Position', obj.States, 'uni', 0));
        end

        function res = GetVelocities(obj)
            res = cell2mat(cellfun(@(s)s.Velocity', obj.States, 'uni', 0));
        end

        function res = GetRPYs(obj)
            res = cell2mat(cellfun(@(s)s.RPY', obj.States, 'uni', 0));
        end

        function res = GetOmegas(obj)
            res = cell2mat(cellfun(@(s)s.Omega', obj.States, 'uni', 0));
        end

        function res = GetForces(obj)
            res = cell2mat(cellfun(@(s)s.Force', obj.States, 'uni', 0));
        end

        function res = GetMoments(obj)
            res = cell2mat(cellfun(@(s)s.Moment', obj.States, 'uni', 0));
        end
        
        function res = GetRotorSpeeds(obj)
            res = cell2mat(cellfun(@(s)s.RotorSpeeds', obj.States, 'uni', 0));
        end
        
        function res = GetRotorSaturations(obj)
            res = cell2mat(cellfun(@(s)s.RotorsSaturated', obj.States, 'uni', 0));
        end
        
        function [res, labels] = GetField(obj, str)
            str = lower(str);
            
            if contains(str, 'accel') && ~contains_or(str, {'ang', 'rot'})
                res = obj.GetAccelerations();
                labels = {'a_x', 'a_y', 'a_z', 'Acceleration'};
                
            elseif contains_or(str, {'euler', 'rpy', 'att', 'phi'}) && ...
                    contains_or(str, {'deriv', 'dot', 'vel', 'speed', 'rate'})
                res = obj.GetEulerDerivatives();
                labels = {'Roll Rate', 'Pitch Rate', 'Yaw Rate', 'Attitude Rate'};

            elseif (contains_or(str, {'ang', 'rot'}) && contains(str, 'accel')) || ...
                    (contains_or(str, {'dot', 'deriv'}) && contains(str, 'omega'))
                res = obj.GetAngularAccelerations();
                labels = {'$\dot{\omega}_x$', '$\dot{\omega}_y$', '$\dot{\omega}_z$', 'Angular Acceleration'};

            elseif contains_or(str, {'euler', 'rpy', 'att', 'phi'})
                res = obj.GetRPYs();
                labels = {'Roll', 'Pitch', 'Yaw', 'Attitude'};

            elseif contains(str, 'rpm') || (contains_or(str, {'mot', 'rotor'}) && contains_or(str, {'vel', 'rate', 'speed'}))
                res = obj.GetRotorSpeeds();
                n_rotors = size(res, 2);
                labels = cell(1, n_rotors + 1);
                for i = 1 : n_rotors
                    labels{i} = ['Motor ' num2str(i, '%d')];
                end
                labels{n_rotors + 1} = 'Motor Velocity';

            elseif contains(str, 'omega') || (contains_or(str, {'ang', 'rot'}) && contains_or(str, {'vel', 'rate', 'speed'}))
                res = obj.GetOmegas();
                labels = {'\omega_x', '\omega_y', '\omega_z', 'Angular Velocity'};

            elseif contains(str, 'forc')
                res = obj.GetForces();
                labels = {'F_x', 'F_y', 'F_z', 'Generated Force'};

            elseif contains_or(str, {'momen', 'torq'})
                res = obj.GetMoments();
                labels = {'M_x', 'M_y', 'M_z', 'Generated Moment'};

            elseif contains_or(str, {'vel', 'speed'})
                res = obj.GetVelocities();
                labels = {'V_x', 'V_y', 'V_z', 'Velocity'};
            
            elseif contains(str, 'pos')
                res = obj.GetPositions();
                labels = {'x', 'y', 'z', 'Position'};

            elseif contains(str, 'sat')
                res = obj.GetRotorSaturations();
                labels = {'Motor Saturations', 'Motor Saturations'};

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
