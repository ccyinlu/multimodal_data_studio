% test linefit segmentation
close all;
clc;

addpath('../mex');
addpath('../func');

if ~exist('data_raw_root')
  % data_raw_root = '/media/bingo/SSD/multi_lidar_calib_data/prescan/scene_motion_plane_sync';
  data_raw_root = '/media/bingo/SSD/multi_lidar_calib_data/sharingVAN/20200706/lidar_calibration';
end

if ~exist('lidar_type')
  lidar_type = 'back';
end

if ~exist('data_index')
  data_index = 1;
end

if ~exist('r_min')
  r_min = 0.2;
end

if ~exist('r_max')
  r_max = 100;
end

if ~exist('n_bins')
  n_bins = 360;
end

if ~exist('n_segments')
  n_segments = 360;
end

if ~exist('max_dist_to_line')
  max_dist_to_line = 0.05;
end

if ~exist('max_slope')
  max_slope = 0.3;
end

if ~exist('max_fit_error')
  max_fit_error = 0.02; % 0.05
end

if ~exist('long_threshold')
  long_threshold = 1.0;
end

if ~exist('max_long_height')
  max_long_height = 0.1;
end

if ~exist('max_start_height')
  max_start_height = 0.2;
end

if ~exist('sensor_height')
  sensor_height = 0.8;
end

if ~exist('line_search_angle')
  line_search_angle = 0.1;
end

if ~exist('n_threads')
  n_threads = 4;
end

if ~exist('leveling')
  leveling = false;
end

if ~exist('levelingPreset')
  levelingPreset = false;
end

if ~exist('levelingPresetZ')
  levelingPresetZ = 0;
end

if ~exist('levelingPresetPitch')
  levelingPresetPitch = 0;
end

if ~exist('levelingPresetRoll')
  levelingPresetRoll = 0;
end

point_cloud_dir = sprintf('%s/lidar_%s', data_raw_root, lidar_type);
point_cloud_filename = sprintf('%s/%06d.pcd', point_cloud_dir, data_index);
input_point_cloud_pt = pcread(point_cloud_filename);
input_point_cloud = double(input_point_cloud_pt.Location);

groundSegmentParams = struct();
groundSegmentParams.r_min_square = double(r_min * r_min);
groundSegmentParams.r_max_square = double(r_max * r_max);
groundSegmentParams.n_bins = double(n_bins);
groundSegmentParams.n_segments = double(n_segments);
groundSegmentParams.max_dist_to_line = double(max_dist_to_line);
groundSegmentParams.max_slope = double(max_slope);
groundSegmentParams.max_error_square = double(max_fit_error * max_fit_error);
groundSegmentParams.long_threshold = double(long_threshold);
groundSegmentParams.max_long_height = double(max_long_height);
groundSegmentParams.max_start_height = double(max_start_height);
groundSegmentParams.sensor_height = double(sensor_height);
groundSegmentParams.line_search_angle = double(line_search_angle);
groundSegmentParams.n_threads = double(n_threads);
groundSegmentParams.leveling = leveling;
groundSegmentParams.levelingPreset = levelingPreset;
groundSegmentParams.levelingPresetZ = levelingPresetZ;
groundSegmentParams.levelingPresetPitch = levelingPresetPitch;
groundSegmentParams.levelingPresetRoll = levelingPresetRoll;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% segment the raw input_point_cloud
tic;
[ground_point_cloud, obstacle_point_cloud] = linefit_ground_segment(groundSegmentParams, input_point_cloud);
toc;

limits = [-100 100 -100 100 -100 100];
Ilimits = [0 255];
fig_ground_segmentation = figure(1);
axes_ground_segmentation = axes(...
                        'Parent', fig_ground_segmentation, ...
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
axis(axes_ground_segmentation, 'equal');
set(fig_ground_segmentation, 'Color', [0.25 0.25 0.25]);
set(axes_ground_segmentation, 'Color', [0.25 0.25 0.25]);
set(axes_ground_segmentation, 'Box', 'off');
set(axes_ground_segmentation, 'XColor', [0.25 0.25 0.25]);
set(axes_ground_segmentation, 'YColor', [0.25 0.25 0.25]);
set(axes_ground_segmentation, 'ZColor', [0.25 0.25 0.25]);

pointsShow(axes_ground_segmentation, ground_point_cloud, limits, Ilimits, 'plain', 3, [0, 0, 0]);
hold on;
pointsShow(axes_ground_segmentation, obstacle_point_cloud, limits, Ilimits, 'plain', 3, [1, 1, 1]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% segment the leveling input_point_cloud using the preset params
groundSegmentParams.leveling = true;
groundSegmentParams.levelingPreset = true;
groundSegmentParams.levelingPresetZ = 1.25;
groundSegmentParams.levelingPresetRoll = 1/180*pi; % 1 degree
groundSegmentParams.levelingPresetPitch = 3/180*pi; % 1 degree
tic;
[ground_point_cloud, obstacle_point_cloud] = linefit_ground_segment(groundSegmentParams, input_point_cloud);
toc;

limits = [-100 100 -100 100 -100 100];
Ilimits = [0 255];
fig_ground_segmentation_levelingPreset = figure(2);
axes_ground_segmentation_levelingPreset = axes(...
                        'Parent', fig_ground_segmentation_levelingPreset, ...
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
axis(axes_ground_segmentation_levelingPreset, 'equal');
set(fig_ground_segmentation_levelingPreset, 'Color', [0.25 0.25 0.25]);
set(axes_ground_segmentation_levelingPreset, 'Color', [0.25 0.25 0.25]);
set(axes_ground_segmentation_levelingPreset, 'Box', 'off');
set(axes_ground_segmentation_levelingPreset, 'XColor', [0.25 0.25 0.25]);
set(axes_ground_segmentation_levelingPreset, 'YColor', [0.25 0.25 0.25]);
set(axes_ground_segmentation_levelingPreset, 'ZColor', [0.25 0.25 0.25]);

pointsShow(axes_ground_segmentation_levelingPreset, ground_point_cloud, limits, Ilimits, 'plain', 3, [0, 0, 0]);
hold on;
pointsShow(axes_ground_segmentation_levelingPreset, obstacle_point_cloud, limits, Ilimits, 'plain', 3, [1, 1, 1]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% segment the leveling input_point_cloud using the ransac estimated ground plane
[mount_z, mount_pitch, mount_roll] = linefit_ground_estimation(groundSegmentParams, input_point_cloud);
groundSegmentParams.leveling = true;
groundSegmentParams.levelingPreset = true;
groundSegmentParams.levelingPresetZ = mount_z;
groundSegmentParams.levelingPresetRoll = mount_roll/180*pi; % degree
groundSegmentParams.levelingPresetPitch = mount_pitch/180*pi; % degree
tic;
[ground_point_cloud, obstacle_point_cloud] = linefit_ground_segment(groundSegmentParams, input_point_cloud);
toc;

limits = [-100 100 -100 100 -100 100];
Ilimits = [0 255];
fig_ground_segmentation_levelingEstimated = figure(3);
axes_ground_segmentation_levelingEstimated = axes(...
                        'Parent', fig_ground_segmentation_levelingEstimated, ...
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
axis(axes_ground_segmentation_levelingEstimated, 'equal');
set(fig_ground_segmentation_levelingEstimated, 'Color', [0.25 0.25 0.25]);
set(axes_ground_segmentation_levelingEstimated, 'Color', [0.25 0.25 0.25]);
set(axes_ground_segmentation_levelingEstimated, 'Box', 'off');
set(axes_ground_segmentation_levelingEstimated, 'XColor', [0.25 0.25 0.25]);
set(axes_ground_segmentation_levelingEstimated, 'YColor', [0.25 0.25 0.25]);
set(axes_ground_segmentation_levelingEstimated, 'ZColor', [0.25 0.25 0.25]);

pointsShow(axes_ground_segmentation_levelingEstimated, ground_point_cloud, limits, Ilimits, 'plain', 3, [0, 0, 0]);
hold on;
pointsShow(axes_ground_segmentation_levelingEstimated, obstacle_point_cloud, limits, Ilimits, 'plain', 3, [1, 1, 1]);