function [bag, index] = loadRosbags(bagFilePath,bags)
    % load rosbag acoording to the bag filepath
    % as the load operation will take for a while, especially when the file size is large
    % so first check whether the bag variable empty in the workspace and then check the file path

    % @Author: ethan
    % @Email: ccyinlu@whu.edu.cn
    % @Date: 2018-10-30

    ifBagLoaded = 0;
    index = 0;

    bagsNum = length(bags);
    if bagsNum
        for i = 1 : bagsNum
            if strcmp(bags{i}.FilePath, bagFilePath)
                str = sprintf('%s already loaded', bagFilePath);
                disp(myLog(str));
                ifBagLoaded = 1;
                index = i;
                bag = bags{i};
            end
        end
    end

    if ~ifBagLoaded
        str = sprintf('%s not loaded', bagFilePath);
        disp(myLog(str));
        disp(myLog('reading the bagfile... please wait...'));
        bag = rosbag(bagFilePath);
    end
end
