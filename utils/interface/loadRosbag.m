function bag = loadRosbag(bagFilePath,bag)
    % load rosbag acoording to the bag filepath
    % as the load operation will take for a while, especially when the file size is large
    % so first check whether the bag variable empty in the workspace and then check the file path

    % @Author: ethan
    % @Email: ccyinlu@whu.edu.cn
    % @Date: 2018-10-30

    ifBagLoaded = 0;

    if ~isempty(bag)
        if strcmp(bag.FilePath, bagFilePath)
            disp(myLog('bag already loaded'));
            ifBagLoaded = 1;
        end
    end

    if ~ifBagLoaded
        disp(myLog('bag not loaded'));
        disp(myLog('reading the bagfile... please wait...'));
        bag = rosbag(bagFilePath);
    end
end
