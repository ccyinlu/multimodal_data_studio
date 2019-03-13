function num = getNumFromTopics(rosbag, topicName)
%GETNUMFROMTOPICS Summary of this function goes here
%   Detailed explanation goes here
    availableTopicsTable = rosbag.AvailableTopics;
    
    cellTopicNum = table2cell(availableTopicsTable(:,1));
    cellTopicNames = availableTopicsTable.Properties.RowNames;
    num = 0;
    for index = 1:size(cellTopicNum,1)
        if isequal(cellTopicNames{index}, topicName)
            num = cellTopicNum{index};
            break;
        end
    end
end

