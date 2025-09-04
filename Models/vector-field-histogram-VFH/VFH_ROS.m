classdef VFH_ROS < matlab.System
    properties
        DistanceLimits = [0.2, 1];
        RobotRadius = 0.3;
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

        function steeringDir = stepImpl(obj, scanMsg, targetDir)

            %Scan
            ranges = double(scanMsg.ranges(:));    
            angleMin = scanMsg.angle_min;
            angleIncrement = scanMsg.angle_increment;
             
            angles = angleMin + (0:length(ranges)-1)' * angleIncrement;     %%%%%%%%%Must be changed%%%%%%%%
            scan = lidarScan(ranges, angles); 

            %Target Direction
            steeringDir = obj.vfh(scan, targetDir);
        end

        function resetImpl(obj)
            reset(obj.vfh);
        end

        function num = getNumInputsImpl(~)
            num = 2;
        end

        function num = getNumOutputsImpl(~)
            num = 1;
        end

        function ds = getDiscreteStateImpl(~)
            ds = struct([]);
        end

        function flag = isInputSizeMutableImpl(~, ~)
            flag = false;
        end

        function flag = isInputDataTypeMutableImpl(~, ~)
            flag = false;
        end
    end
end