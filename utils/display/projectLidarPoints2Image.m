function projectLidarPoints2Image(ax, I, pointcloud, pointcloud_intensity, K, R, T, renderMode, limits, Ilimits, pointSize)
    % inputs:
    % ax, the axes of the canvas
    % I = image from image topic
    % pointcloud = 3D points [3xn] in lidar reference frame for this part image 
    % pointcloud_intensity, intensity of the point cloud
    % K = internal camera matrix of this sensor
    % R = rotation matrix for lidar reference frame to camera reference frame
    % T = translation matrix for lidar reference frame to camera reference frames

    % renderMode = render mode of the projected points, it can be set with 'plain', 'range', 'height', 'intensity'
    % limits, limitation of the pointcloud
    % Ilimits, limitation of the pointcloud related to intensity
    % pointSize, the size of the rendered projected point

    %% set the colormap
    CC = hsv(256);

    col = size(I,2);
    row = size(I,1);

    %% crop the points according to the limits
    indexValid = pointcloud(1, :) > limits(1) & pointcloud(1, :) < limits(2) & pointcloud(2, :) > limits(3) & pointcloud(2, :) < limits(4) & pointcloud(3, :) > limits(5) & pointcloud(3, :) < limits(6);
    pointcloud = pointcloud(:, indexValid);
    if ~isempty(pointcloud_intensity)
        pointcloud_intensity = pointcloud_intensity(indexValid);
    end

    Pc = R*pointcloud + T;

    % the points project to the back of the camera will be removed
    valid_index = Pc(3,:) > 1;
    Pc = Pc(:, valid_index);
    pointcloud = pointcloud(:, valid_index);
    if ~isempty(pointcloud_intensity)
        pointcloud_intensity = pointcloud_intensity(valid_index);
    end

    image_points = K*Pc;
    tempX = image_points(1,:)./ image_points(3,:);
    tempY = image_points(2,:)./ image_points(3,:);

    index = find(tempX(1,:) > 1 & tempX(1,:) < col &  tempY(1,:) > 1 & tempY(1,:) < row); 
    X_final = tempX(index);
    Y_final = tempY(index);

    pointcloud_final = (pointcloud(:, index))';
    if ~isempty(pointcloud_intensity)
        pointcloud_intensity = pointcloud_intensity(index);
    end

    if isequal(renderMode, 'plain')
        %% plain color, set the color to 'blue'
        C = 'r';
    elseif isequal(renderMode, 'height')
        %% set the color according to the height
        C = CC(floor((pointcloud_final(:, 3) - limits(5))/(limits(6) - limits(5)) * 255 + 1), :);
    elseif isequal(renderMode, 'range')
        %% set the color according to the range
        xRange = max(abs(limits(1)), abs(limits(2)));
        yRange = max(abs(limits(3)), abs(limits(4)));
        zRange = max(abs(limits(5)), abs(limits(6)));
        r = sqrt(pointcloud_final(:, 1).^2 + pointcloud_final(:, 2).^2 + pointcloud_final(:, 3).^2);
        r_max = sqrt(xRange.^2 + yRange.^2 + zRange.^2);
        C = CC(floor(r/r_max * 255 + 1), :);
    elseif isequal(renderMode, 'intensity')
        %% set the color according to the intensity
        if ~isempty(pointcloud_intensity)
            intensity = pointcloud_intensity;
            intensityLow = max(min(intensity), Ilimits(1));
            intensityHigh = min(std(intensity) * 3 + mean(intensity), Ilimits(2));

            intensity(intensity <= intensityLow) = intensityLow; 
            intensity(intensity >= intensityHigh) = intensityHigh; 
            C = CC(floor((intensity - intensityLow)/(intensityHigh - intensityLow) * 255 + 1), :);
        else
            C = 'r';
        end
    end

    scatter(ax, X_final, Y_final, pointSize, C, 'filled'); 
end