function topics = getTopicsByType(rosbag, messageType)
%GETTOPICSBYTYPE Summary of this function goes here
%   Detailed explanation goes here
    
    topics = [];
    availableTopicsTable = rosbag.AvailableTopics;
    % convert the conlum of MessageType to cell array
    cellTopicTypes = table2cell(availableTopicsTable(:,2));
    cellTopicNames = availableTopicsTable.Properties.RowNames;
    k = 1;
    for index = 1:size(cellTopicTypes,1)
        if isequal(messageType, '')
            topics{k} = cellTopicNames{index};
            k = k + 1;
        else
            if isequal(cellTopicTypes{index}, messageType)
                topics{k} = cellTopicNames{index};
                k = k + 1;
            end
        end
    end
end

