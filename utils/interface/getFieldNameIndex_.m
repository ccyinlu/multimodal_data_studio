function fieldIdx = getFieldNameIndex_(obj, fieldName)
    %getFieldNameIndex Get index of field in PointField array

    allFieldNames = readAllFieldNames_(obj);
    [isValid, fieldIdx] = ismember(fieldName, allFieldNames);
    if ~isValid
        error(message('robotics:ros:pointcloud:InvalidFieldName', ...
            fieldName, strjoin(allFieldNames, ', ')));
    end
end