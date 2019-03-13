function [message, timeOffset] = getMessageTopicTimeNearest(obj, timestamps, time, timeOffsetThreshold)
    %   get the message from the bag object according to the topic and time
        % find the nearest time within the gap between the  timeOffsetThreshold ~ time ~ timeOffsetThreshold
        % timeOffset will record the true gap between the selected message time and the specified time
        % if the timeOffset larger than the timeOffsetThreshold, then the message will be specified to empty
    
        % Example
        % [lidarMessage, timeOffset] = getMessageTopicTimeNearest(bag, '/lidar0/points_raw', startTime, 0.5);
    
        % get all the timestamps according to the topic
        [~, I] = min(abs(timestamps(:) - time));
    
        timeSelected = timestamps(I);
        timeOffset = time - timeSelected;
        if abs(timeOffset) > timeOffsetThreshold
            message = [];
            timeOffset = 0;
            return;
        else
            if I < length(timestamps)
                if (timeSelected + timestamps(I + 1))/2 < timestamps(end)
                    topicSelected = select(obj, 'Time', [timeSelected (timeSelected + timestamps(I + 1))/2]);
                    messages = readMessages(topicSelected, 1);
                    message = messages{1};
                else
                    message = [];
                    timeOffset = 0;
                end
            else
                message = [];
                timeOffset = 0;
            end
        end
    
    end