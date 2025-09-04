classdef VFHSystem < matlab.System
    properties
        DistanceLimits = [0.2, 2];
        RobotRadius = 0.2;
        SafetyDistance = 0.2;
        MinTurningRadius = 0.3;
    end

    properties (Access = private)
        vfh;
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.vfh = controllerVFH('DistanceLimits', obj.DistanceLimits, ...
                                    'RobotRadius', obj.RobotRadius, ...
                                    'SafetyDistance', obj.SafetyDistance, ...
                                    'MinTurningRadius', obj.MinTurningRadius, ...
                                    'UseLidarScan', true);
        end

        function steeringDir = stepImpl(obj, ranges, angles, targetDir)
            scan = lidarScan(ranges, angles);
            steeringDir = obj.vfh(scan, targetDir);
        end

    end
end
