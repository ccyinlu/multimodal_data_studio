function [messages, timeOffsets, founds] = getSyncMessagesFromTopics(obj, topics, time, timeOffsetThresholds)
    %   get the Synced messages from the bag object according to the topics and time
        % find the nearest messages within the gap between the  timeOffsetThresholds[i] ~ time ~ timeOffsetThreshold[i]
        % timeOffsets will record the true gap between the selected message time and the specified time
        % if the timeOffsets larger than the timeOffsetThresholds, then the message will be specified to empty, and founds set to zeros
    
        % Example
        % [syncMessages, timeOffsets] = getSyncMessagesFromTopics(bag, {'/lidar0/points_raw', '/usb_cam/image_raw'}, startTime, [0.5, 0.6]);
    
        % get all the timestamps according to the topic
        topicsNum = size(topics, 2);
        messages = cell(1, topicsNum);
        timeOffsets = zeros(1, topicsNum);
        founds = zeros(1, topicsNum);
        for i = 1:topicsNum
            topicsAll = select(obj, 'Topic', topics{i});
            timeStamps = topicsAll.MessageList.Time;
            [~, I] = min(abs(timeStamps(:) - time));
            timeSelected = timeStamps(I);
            timeOffsets(i) = timeSelected - time;
            if abs(timeOffsets(i)) > timeOffsetThresholds(i)
                messages{i} = [];
                founds(i) = 0;
                continue;
            else
                topicSelected = select(obj, 'Topic', topics{i}, 'Time', [timeSelected timeSelected + timeOffsetThresholds(i)]);
                messagesSelected = readMessages(topicSelected, 1);
                messages{i} = messagesSelected{1};
                founds(i) = 1;
            end
        end
    end