classdef GridParams
    properties
        g0;
        x;
        y;
        l;
        centers;
        x0_size;
        ur5;
    end
    methods
        function obj = GridParams(g0, x, y, l, centers, x0_size, ur5)
            obj.g0 = g0;
            obj.x = x;
            obj.y = y;
            obj.l = l;
            obj.centers = centers;
            obj.x0_size = x0_size;
            obj.ur5 = ur5;
        end
    end
end
