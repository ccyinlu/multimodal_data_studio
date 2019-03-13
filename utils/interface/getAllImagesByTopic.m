function images = getAllImagesByTopic(obj, topic)
%GETALLIMAGESBYTOPIC Summary of this function goes here
%   Detailed explanation goes here
    topicSelected = select(obj, 'Topic', topic);
    messages = readMessages(topicSelected);
    images = cell(size(messages));
    for index = 1 : length(messages)
        images{index} = readImage(images{index});
    end
end

