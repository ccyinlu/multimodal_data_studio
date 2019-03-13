function [byteIdx, pointIdxIsValid] = getByteIndexForField_(obj, fieldIdx, pointIndices, count)
            %getByteIndexForField Get a vector of bytes indices for field
            %at specific points
            
            if nargin < 3
                pointIndices = 1:obj.Height*obj.Width;
            end
            if nargin < 4
                count = getFieldCount_(obj, fieldIdx);
            end
            
            % Number of points requested
            numPoints = numel(pointIndices);
            
            % Compute actual number of available points (accounting for
            % potential truncation)
            numPointsActual = min(obj.Height*obj.Width, fix(numel(obj.Data)/double(obj.PointStep)));
            
            % Recover field offset and MATLAB data type of field elements
            offset = getFieldOffset_(obj, fieldIdx);
            datatype = getFieldDatatype_(obj, fieldIdx);
            [~, numBytes] = rosToMATLABType_(datatype);
            numBytes = numBytes * double(count);
            
            % Extract the bytes corresponding to this field for all points
            pointStep = obj.PointStep;
            
            byteIdx = zeros(numPoints, numBytes);
            pointIdxIsValid = (0 < pointIndices) & (pointIndices <= numPointsActual);
            validPointIndices = pointIndices(pointIdxIsValid);
            startIndices = double(offset ...
                + pointStep*cast(validPointIndices(:)-1,'like',pointStep));
            byteIdx(pointIdxIsValid,:) = bsxfun(@plus, startIndices, 1:numBytes);
            if nargout < 2
                byteIdx = byteIdx(pointIdxIsValid,:);
            end
        end