classdef controller
    properties
        ControlAllocation control_allocation
    end
    
    methods
        function obj = controller(multirotor)
            obj.ControlAllocation = control_allocation(multirotor);
        end
    end
end

