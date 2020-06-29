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
            
%             obj.GetVelocities()
%             obj.GetRPYs()
%             obj.GetOmegas()
%             obj.GetForces()
%             obj.GetMoments()
%             obj.GetRotorSpeeds()
%             obj.GetRotorSaturations()
% 
            if contains(str, 'accel') && ~contains(str, 'ang') && ~contains(str, 'rot')
                res = obj.GetAccelerations();
                labels = {'a_x', 'a_y', 'a_z', 'Acceleration'};
                
            elseif (contains(str, 'euler') || contains(str, 'rpy') || contains(str, 'att') || contains(str, 'phi')) && ...
                    (contains(str, 'deriv') || contains(str, 'dot') || contains(str, 'vel') || contains(str, 'speed'))
                res = obj.GetEulerDerivatives();
                labels = {'Roll Rate', 'Pitch Rate', 'Yaw Rate', 'Euler Derivative'};

            elseif ((contains(str, 'ang') || contains(str, 'rot')) && contains(str, 'accel')) || ...
                    ((contains(str, 'dot') || contains(str, 'deriv')) && contains(str, 'omega'))
                res = obj.GetAngularAccelerations();
                labels = {'\omega_x', '\omega_y', '\omega_z', 'Angular Acceleration'};

            elseif contains(str, 'pos')
                res = obj.GetPositions();
                labels = {'x', 'y', 'z', 'Position'};

            
            else
                error('Input string not recognized.');
            end
        end
        
    end
end
