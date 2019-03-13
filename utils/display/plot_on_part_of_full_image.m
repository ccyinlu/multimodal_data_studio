function plot_on_part_of_full_image(ax, I, pointcloud, K, R, T, sort_mode, limits, height_min, pointSize)
    %% This function projects the pointcloud on the corresponding part of full
    % @Author: ethan
    % @Email: ccyinlu@whu.edu.cn
    % @Date: 2018-10-30

    % inputs:
    % I = image from image topic
    % pointcloud = 3D points [3xn] in lidar reference frame for this part image 
    % K = internal camera matrix of this sensor
    % R = rotation matrix for lidar reference frame to camera reference frame
    % T = translation matrix for lidar reference frame to camera reference frames
    % sort_mode

    numColors = 100;
    if nargin < 7
        height_min = -2.5;
    end

    col = size(I,2);
    row = size(I,1);

    %% crop the points according to the limits
    indexValid = pointcloud(1, :) > limits(1) & pointcloud(1, :) < limits(2) & pointcloud(2, :) > limits(3) & pointcloud(2, :) < limits(4) & pointcloud(3, :) > limits(5) & pointcloud(3, :) < limits(6);
    pointcloud = pointcloud(:, indexValid);

    Pc = R*pointcloud + T;

    % the points project to the back of the camera will be removed
    valid_index = Pc(3,:) > 1;
    Pc = Pc(:, valid_index);
    pointcloud = pointcloud(:, valid_index);

    image_points = K*Pc;
    tempX = image_points(1,:)./ image_points(3,:);
    tempY = image_points(2,:)./ image_points(3,:);

    %Image points sorted with Z/range coordinate
    if(sort_mode == 'range') %sort based on range from camera
        %range of points in lidar frame
        range_lidar = pointcloud.*pointcloud;
        range = sqrt(sum(range_lidar,1));
        [sorted_range ii_r] = sort(range);

        tempX = tempX(ii_r);
        tempY = tempY(ii_r);
        temp_min = 0.5;
        temp_max = 80;
        step = (temp_max - temp_min)/numColors; % in meters
    elseif(sort_mode == 'height') % sort based on height above the ground plane
        %Height above the ground plane 
        H_lidar = pointcloud(3,:);
        [sorted_Z ii_z] = sort(H_lidar);

        tempX = tempX(ii_z);
        tempY = tempY(ii_z);
        temp_min = height_min;
        temp_max = 10;
        step = (temp_max - temp_min)/numColors;
    end

    index = find(tempX(1,:) > 1 & tempX(1,:) < col &  tempY(1,:) > 1 & tempY(1,:) < row); 
    X_final = tempX(index);
    Y_final = tempY(index);

    if(sort_mode == 'range')
        sorted = sorted_range(index);
    elseif(sort_mode == 'height')
        sorted = sorted_Z(index);
    end

    n = size(X_final,2);

    color = hsv(numColors);

    %Project the points with different colors
    for i = 1:numColors
        temp_index = find(sorted(:) >= temp_min & sorted(:) < (temp_min + step));
        temp_X_final = X_final(temp_index);
        temp_Y_final = Y_final(temp_index);
        temp_min = temp_min + step;
        scatter(ax, temp_X_final, temp_Y_final, pointSize, color(i,:), 'filled'); 
    end
end