% 
clear all;
close all;clc;

addpath('/usr/local/lib/libopencv_core.so.3.3');
addpath('../mex');

test_lidar_type = 'titan';

titan_root_dir = '/media/hy/DATA2/ethan/titan/experiment_wuhan_university_01/lidar0';
% kitti_root_dir = '/media/hy/DATA2/KITTI/object_3d/training/velodyne';
kitti_root_dir = '/media/hy/DATA2/KITTI/odometry/sequence_00/velodyne';

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

    params.groundScanInd = 12;
    params.sensorMountAngle = 0;
    params.groundRemovalAngleT = 2;
    params.segmentTheta = 20/180.0*pi;
    params.feasibleSegmentValidPointNum = 60; % 30
    params.segmentValidPointNum = 10; % 5
    params.segmentValidLineNum = 6; % 3
    params.debugInfo = [18 7 14];

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

    params.groundScanInd = 8;
    params.sensorMountAngle = 0;
    params.groundRemovalAngleT = 10;

    params.segmentTheta = 60/180.0*pi;
    params.feasibleSegmentValidPointNum = 30; % 30
    params.segmentValidPointNum = 5; % 5
    params.segmentValidLineNum = 3; % 3
    params.debugInfo = [18 7 233];

end

tic
    % you should convert the points from single to double, if you use the pcread, then the point type will be single
    [rangeMat, rangeMatFilled] = projectPointCloudMex(params, double(points));
toc

% groundLabel
% -1, no valid info to check if ground of not
% 0, initial value, after validation, means not ground
% 1, ground

% tic
%     groundLabel = groundRemovalMex(params, double(rangeMat));
% toc

tic
    groundLabelFilled = groundRemovalMex(params, double(rangeMatFilled));
toc

% tic
%     [labelMat] = cloudSegmentationMex(params, double(rangeMat), int8(groundLabel));
% toc

tic
    [labelMatFilled] = cloudSegmentationMex(params, double(rangeMatFilled), int8(groundLabelFilled));
toc

limits = [-100 100 -100 100 -3 2];
Ilimits = [0 255];

% fig_segment_cloud = figure(1);
% axes_segment_cloud = axes(...
%                         'Parent', fig_segment_cloud, ...
%                         'Units', 'Normalized', ...
%                         'Position', [0 0 1 1], ...
%                         'XTickLabel', '', ...
%                         'YTickLabel', '', ...
%                         'ZTickLabel', '', ...
%                         'XTick', '', ...
%                         'YTick', '', ...
%                         'ZTick', '', ...
%                         'Color', [0.25 0.25 0.25], ...
%                         'PickableParts', 'all', ...
%                         'HitTest', 'on', ...
%                         'NextPlot', 'replacechildren', ...
%                         'Visible', 'on', ...
%                         'XLimMode', 'auto', ...
%                         'YLimMode', 'auto', ...
%                         'ZLimMode', 'auto', ...
%                         'DataAspectRatioMode', 'auto', ...
%                         'PlotBoxAspectRatioMode', 'auto');

% % axis the point view equal
% set(fig_segment_cloud, 'Color', [0.25 0.25 0.25]);
% axis(axes_segment_cloud, 'equal');
% set(axes_segment_cloud, 'Color', [0.25 0.25 0.25]);
% set(axes_segment_cloud, 'Box', 'off');
% set(axes_segment_cloud, 'XColor', [0.25 0.25 0.25]);
% set(axes_segment_cloud, 'YColor', [0.25 0.25 0.25]);
% set(axes_segment_cloud, 'ZColor', [0.25 0.25 0.25]);

% labels = unique(labelMat);
% color_num = length(labels) - 1;

% colors = jet(color_num);
% result = reshape(rangeMat(:, :, 3:6), [], 4);

% pointsShow(axes_segment_cloud, result(:, :), limits, Ilimits, 'plain', 3, [1 1 1]);
% hold(axes_segment_cloud, 'on');

% % color the ground point as black
% ground_index = find(groundLabel == 1);
% pointsShow(axes_segment_cloud, result(ground_index, :), limits, Ilimits, 'plain', 3, [0 0 0]);
% hold(axes_segment_cloud, 'on');

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % render the labels
% for i = 1 : length(labels) - 1
%     label_color = colors(i, :);
%     label_index = find(labelMat == labels(i));
%     pointsShow(axes_segment_cloud, result(label_index, :), limits, Ilimits, 'plain', 3, label_color);
%     hold(axes_segment_cloud, 'on');
% end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fig_segment_cloud_filled = figure(2);
axes_segment_cloud_filled = axes(...
                        'Parent', fig_segment_cloud_filled, ...
                        'Units', 'Normalized', ...
                        'Position', [0 0 1 1], ...
                        'XTickLabel', '', ...
                        'YTickLabel', '', ...
                        'ZTickLabel', '', ...
                        'XTick', '', ...
                        'YTick', '', ...
                        'ZTick', '', ...
                        'Color', [0.25 0.25 0.25], ...
                        'PickableParts', 'all', ...
                        'HitTest', 'on', ...
                        'NextPlot', 'replacechildren', ...
                        'Visible', 'on', ...
                        'XLimMode', 'auto', ...
                        'YLimMode', 'auto', ...
                        'ZLimMode', 'auto', ...
                        'DataAspectRatioMode', 'auto', ...
                        'PlotBoxAspectRatioMode', 'auto');

% axis the point view equal
set(fig_segment_cloud_filled, 'Color', [0.25 0.25 0.25]);
axis(axes_segment_cloud_filled, 'equal');
set(axes_segment_cloud_filled, 'Color', [0.25 0.25 0.25]);
set(axes_segment_cloud_filled, 'Box', 'off');
set(axes_segment_cloud_filled, 'XColor', [0.25 0.25 0.25]);
set(axes_segment_cloud_filled, 'YColor', [0.25 0.25 0.25]);
set(axes_segment_cloud_filled, 'ZColor', [0.25 0.25 0.25]);

labels = unique(labelMatFilled);
color_num = length(labels) - 1;

colors = jet(color_num);
result = reshape(rangeMatFilled(:, :, 3:6), [], 4);

pointsShow(axes_segment_cloud_filled, result(:, :), limits, Ilimits, 'plain', 3, [1 1 1]);
hold(axes_segment_cloud_filled, 'on');

% color the ground point as black
ground_index = find(groundLabelFilled == 1);
pointsShow(axes_segment_cloud_filled, result(ground_index, :), limits, Ilimits, 'plain', 3, [0 0 0]);
hold(axes_segment_cloud_filled, 'on');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1 : length(labels) - 1
    label_color = colors(i, :);
    label_index = find(labelMatFilled == labels(i));
    pointsShow(axes_segment_cloud_filled, result(label_index, :), limits, Ilimits, 'plain', 3, label_color);
    hold(axes_segment_cloud_filled, 'on');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

