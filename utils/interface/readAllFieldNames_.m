function fieldNames = readAllFieldNames_(obj)
            %readAllFieldNames - Return all available field names
            %   FIELDNAMES = readAllFieldNames(OBJ) returns the names of
            %   all point fields that are stored in message OBJ. FIELDNAMES
            %   is a 1xN cell array of strings, where N is the number of fields.
            %   If no fields are stored in the message, the return will be
            %   an empty cell array.
            
            % Some of the allowable field names are documented on the ROS
            % Wiki: http://wiki.ros.org/pcl/Overview#Common_PointCloud2_field_names
            
            fieldNames = {};
            
            % If no point fields are available, return
            if isempty(obj.Fields)
                return;
            end
            
            fieldNames = {obj.Fields.Name};
 end