function lidarPoints = image2lidar(imagePoints, CameraExtrinsicMat, CameraMat)
    % convert the points coordinates in the camera reference frame to the lidar planar frame
    % as for a 2D point in the image plare, it corresponds a ray in the lidar coordinate frame, so we can define the scale factor as 1

    % inputs: 
    % imagePoints = 2D points [2xn] in camera reference frame 
    % CameraExtrinsicMat = transformation matrix for camera reference frame to lidar reference frame
    % CameraMat = intrinsic matrix of the camera

    % outputs:
    % lidarPoints = 3D points [3xN] in lidar reference frame

    % @Author: ethan
    % @Email: ccyinlu@whu.edu.cn
    % @Date: 2018-10-30

    z = 5;

    % transformation from lidar to camera
    R_c2l = CameraExtrinsicMat(1:3, 1:3);
    T_c2l = CameraExtrinsicMat(1:3, 4);
    fx = CameraMat(1, 1);
    fy = CameraMat(2, 2);
    u0 = CameraMat(1, 3);
    v0 = CameraMat(2, 3);

    imagePoints_x = z * (imagePoints(1, :) - u0)/fx;
    imagePoints_y = z * (imagePoints(2, :) - v0)/fy;
    imagePoints_z = z * ones(1, size(imagePoints, 2));

    imagePoints_3D = [imagePoints_x; imagePoints_y; imagePoints_z];
    
    lidarPoints = R_c2l*imagePoints_3D + T_c2l;
    %%% adjust the Extrinsic Matrix especially the T for a little
    % lidarPoints(3, :) = lidarPoints(3, :) + 0.4;
    % lidarPoints(1, :) = lidarPoints(1, :) + 0.1;
end