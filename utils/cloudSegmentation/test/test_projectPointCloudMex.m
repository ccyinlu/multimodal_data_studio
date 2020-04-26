% 
clear all;
close all;clc;
addpath('../mex');
test_lidar_type = 'titan';

titan_root_dir = '/media/hy/DATA2/ethan/titan/experiment_wuhan_university_01/lidar0';
kitti_root_dir = '/media/hy/DATA2/KITTI/object_3d/training/velodyne';

idx = 450;
params = struct();

if isequal(test_lidar_type, 'titan')
    point_filename = sprintf('%s/%06d.pcd', titan_root_dir, idx + 1);

    % load the points
    ptCloud = pcread(point_filename);
    points = [ptCloud.Location ptCloud.Intensity];

    vertical_theta = [  15.00 11.00 8.00 5.00 3.00 2.00 1.67 1.33 ...
                        1.00 0.67 0.33 0.00 -0.33 -0.67 -1.00 -1.33 ...
                        -1.67 -2.00 -2.33 -2.67 -3.00 -3.33 -3.67 -4.00 ...
                        -4.33 -4.67 -5.00 -5.33 -5.67 -6.00 -7.00 -8.00 ...
                        -9.00 -10.00 -11.00 -12.00 -13.00 -14.00 -19.00 -25.00
                        ];

    vertical_theta = vertical_theta/180*pi;

    % set the params
    params.N_SCAN = 40;
    params.Horizon_SCAN = 1800;
    params.vertical_theta = vertical_theta;
    params.lidar_type = 'hesai_p40p';

elseif isequal(test_lidar_type, 'kitti')
    point_filename = sprintf('%s/%06d.bin', kitti_root_dir, idx);

    % load the points
    fid = fopen(point_filename);
    points = fread(fid,[4 inf],'single')';
    fclose(fid);

    vertical_theta = [  2.000 1.667 1.333 1.000 0.776 0.333 0.000 -0.333 ...
                        -0.667 -1.000 -1.333 -1.667 -2.000 -2.333 -2.666 -3.000 ...
                        -3.333 -3.666 -4.000 -4.333 -4.666 -5.000 -5.333 -5.666 ...
                        -6.000 -6.333 -6.666 -7.000 -7.333 -7.666 -8.000 -8.333 ...
                        -8.833 -9.33 -9.83 -10.33 -10.833 -11.333 -11.833 -12.333 ...
                        -12.833 -13.333 -13.833 -14.333 -14.833 -15.333 -15.833 -16.333 ...
                        -16.833 -17.333 -17.833 -18.333 -18.833 -19.333 -19.833 -20.333 ....
                        -20.833 -21.333 -21.833 -22.333 -22.833 -23.333 -23.833 -24.33
                        ];

    vertical_theta = vertical_theta/180*pi;

    % set the params
    params.N_SCAN = 64;
    params.Horizon_SCAN = 1800;
    params.vertical_theta = vertical_theta;
    params.lidar_type = 'velodyne_hdl_64e';

end

tic
    [rangeMat, rangeMatFilled, rangeMatRaw] = projectPointCloudMex(params, double(points));
toc

figure(1);
pcshow(pointCloud(points(:, 1:3), 'Intensity', points(:, 4)));

figure(2);
pcshow(pointCloud(rangeMatRaw(:, :, 3:5), 'Intensity', rangeMatFilled(:, :, 6)));

figure(3);
pcshow(pointCloud(rangeMat(:, :, 3:5), 'Intensity', rangeMat(:, :, 6)));

figure(4);
pcshow(pointCloud(rangeMatFilled(:, :, 3:5), 'Intensity', rangeMatFilled(:, :, 6)));


