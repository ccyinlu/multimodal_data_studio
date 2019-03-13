function pointsShowLabel(points, limits, labels, labelsNum)
    % show the 3D points according to the labels

    % @Author: ethan
    % @Email: ccyinlu@whu.edu.cn
    % @Date: 2018-10-31

    % inputs: 
    % points: 3xN points or 4xN points in the lidar coordinate frame
    % limits: 1x6 matrix to limits the range of the lidar points
    % labels: 1xN matrix represents the label of the points
    % labelsNum: scalar value, 

    %% set the colormap
    color = hsv(labelsNum);

    %% crop the points according to the limits
    indexValid = points(1, :) > limits(1)...
               & points(1, :) < limits(2)...
               & points(2, :) > limits(3)...
               & points(2, :) < limits(4)...
               & points(3, :) > limits(5)... 
               & points(3, :) < limits(6);
    points = points(:, indexValid);
    labels = labels(indexValid);

    %Project the points with different colors
    for i = 1:labelsNum
        temp_index = find(labels(:) == i);
        temp_X_final = points(1, temp_index);
        temp_Y_final = points(2, temp_index);
        temp_Z_final = points(3, temp_index);
        scatter3(temp_X_final, temp_Y_final, temp_Z_final, 4, color(i,:), 'filled'); 
    end

end