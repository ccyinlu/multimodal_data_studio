function [mlType, numBytes] = rosToMATLABType(type)
    %rosToMATLABType Convert from ROS to MATLAB data type
    %   The data type for each point field is specified by an
    %   integer number. This function converts to the corresponding
    %   MATLAB data type.
    %
    %   See also matlabToROSType.

    % Set conversions between PointCloud2 and MATLAB types
    switch(type)
        case 1      %INT8
            mlType = 'int8';
            numBytes = 1;
        case 2      %UINT8
            mlType = 'uint8';
            numBytes = 1;
        case 3      %INT16
            mlType = 'int16';
            numBytes = 2;
        case 4      %UINT16
            mlType = 'uint16';
            numBytes = 2;
        case 5      %INT32
            mlType = 'int32';
            numBytes = 4;
        case 6      %UINT32
            mlType = 'uint32';
            numBytes = 4;
        case 7      %FLOAT32
            mlType = 'single';
            numBytes = 4;
        case 8      %FLOAT64
            mlType = 'double';
            numBytes = 8;
        otherwise 
            coder.internal.error('robotics:ros:pointcloud:InvalidTypeNum', ...
                type, '1 - 8');
    end
end