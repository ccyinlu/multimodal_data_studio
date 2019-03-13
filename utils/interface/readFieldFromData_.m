function fieldPoints = readFieldFromData_(~, data, byteIdx, pointIdxIsValid, mlType, count)
    %readField Read data based on given field name
    %   This is a generic function for reading data from any field
    %   name specified
    %
    %   This function returns an NxC array of values. N is the
    %   number of points in the point cloud and C is the number of
    %   values that is assigned for every point in this field. In
    %   most cases, C will be 1.
    numPoints = numel(pointIdxIsValid);
    % Initialize output
    rawData = reshape(data(byteIdx(pointIdxIsValid,:)'),[],1);
    validPoints = reshape(typecast(rawData, mlType), count, []).';
    if any(~pointIdxIsValid(:))
        if any(strcmp(mlType, {'single','double'}))
            fieldPoints = NaN(numPoints, count, mlType);
        else
            fieldPoints = zeros(numPoints, count, mlType);
        end
        fieldPoints(pointIdxIsValid, :) = validPoints;
    else
        fieldPoints = validPoints;
    end
end