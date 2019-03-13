function imagePoints = lidar2image(lidarPoints, CameraExtrinsicMat, CameraMat, imageSize)
    % convert the points coordinates in the lidar reference frame to the image planar frame

    % inputs: 
    % lidarPoints = 3D points [3xn] in lidar reference frame for this part image 
    % CameraExtrinsicMat = transformation matrix for camera reference frame to lidar reference frame
    % CameraMat = intrinsic matrix of the camera

    % @Author: ethan
    % @Email: ccyinlu@whu.edu.cn
    % @Date: 2018-10-30

    % transformation from lidar to camera
    R_l2c = CameraExtrinsicMat(1:3, 1:3)';
    T_l2c = -R_l2c * CameraExtrinsicMat(1:3, 4);
    K = CameraMat;

    col = imageSize(1);
    row = imageSize(2);

    Pc = R*lidarPoints + T;

    % the points project to the back of the camera will be removed
    valid_index = Pc(3,:) > 1;
    Pc = Pc(:, valid_index);
    lidarPoints = lidarPoints(:, valid_index);

    image_points = K*Pc;
    tempX = image_points(1,:)./ image_points(3,:);
    tempY = image_points(2,:)./ image_points(3,:);

    index = find(tempX(1,:) > 1 & tempX(1,:) < col &  tempY(1,:) > 1 & tempY(1,:) < row); 
    X_final = tempX(index);
    Y_final = tempY(index);

    imagePoints = [X_final; Y_final];
end