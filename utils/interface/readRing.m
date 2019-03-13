function ring = readRing(obj)
    %   readRing Returns the ring id of all points
    %   ring = readRing(OBJ) extracts the ring id from
    %   all points in the point cloud object OBJ and returns them
    %   as an Nx1 matrix
    %
    %   If the point cloud contains N points, the returned matrix
    %   has Nx1 elements
    
    %try
        % Get field indices for ring
        ringIndex = getFieldNameIndex_(obj, 'ring');
    %             catch ex
    %                 newex = MException(message('pointcloud:InvalidRingData'));
    %                 throw(newex.addCause(ex));
    %             end
    %             
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %readXYZ Returns the (x,y,z) coordinates of all points
    data = obj.Data;
    
    % Recover MATLAB data type of field elements
    RingMlType = rosToMATLABType_(getFieldDatatype_(obj, ringIndex));
    
    pointIndices = 1:obj.Width * obj.Height;
    
    % Get byte index only once (this is expensive)
    [byteIdx, pointIdxIsValid] = getByteIndexForField_(obj, ringIndex, pointIndices, 1);
    
    % Calculate the byte offsets for the different fields
    % This helps with performance, since we can re-use the
    % byte index computed below
    ringOff = double(getFieldOffset_(obj, ringIndex));

    % Retrieve the ring data and concatenate into one matrix
    ring = ...
        [readFieldFromData_(obj, data, byteIdx, pointIdxIsValid, RingMlType,1)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end