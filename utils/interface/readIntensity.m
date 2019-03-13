function intensity = readIntensity(obj)
    %   readIntensity Returns the intensity of all points
    %   intensity = readIntensity(OBJ) extracts the intensity from
    %   all points in the point cloud object OBJ and returns them
    %   as an Nx1 matrix
    %
    %   If the point cloud contains N points, the returned matrix
    %   has Nx1 elements 
    
    %try
    % Get field indices for intensity
    intensityIndex = getFieldNameIndex_(obj, 'intensity');
%             catch ex
%                 newex = MException(message('pointcloud:InvalidRingData'));
%                 throw(newex.addCause(ex));
%             end
%             
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %readXYZ Returns the (x,y,z) coordinates of all points
    data = obj.Data;
    
    % Recover MATLAB data type of field elements
    IntensityMlType = rosToMATLABType_(getFieldDatatype_(obj, intensityIndex));
    
    pointIndices = 1:obj.Width * obj.Height;
    
    % Get byte index only once (this is expensive)
    [byteIdx, pointIdxIsValid] = getByteIndexForField_(obj, intensityIndex, pointIndices, 1);
    
    % Calculate the byte offsets for the different fields
    % This helps with performance, since we can re-use the
    % byte index computed below
    intensityOff = double(getFieldOffset_(obj, intensityIndex));

    % Retrieve the intensity data and concatenate into one matrix
    intensity = ...
        [readFieldFromData_(obj, data, byteIdx, pointIdxIsValid, IntensityMlType,1)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end