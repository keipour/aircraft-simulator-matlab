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
    end
end
