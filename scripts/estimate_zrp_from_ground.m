% extract the ground points and estimate the z, roll and pitch relative to the ground

clc;
close all;

addpath('../thirdParty/yaml');
addpath('../utils/interface');
addpath('../utils/linefit_ground_segmentation/func');
addpath('../utils/linefit_ground_segmentation/mex');

if ~exist('data_root')
  data_root = '/media/bingo/SSD/camera_lidar_calibration/sanheyi/20200817/extri';
end

if ~exist('pointcloud_name')
    pointcloud_name = 'lidar2';
end

start_index = 1;

params_config_file = [data_root '/' 'camera_lidar_calibration_config.yml'];
if ~exist(params_config_file,'file')
    return;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% view options
cameraPosition = [333.113 -2368.436 740.263];
cameraTarget = [-12.925 97.665 -21.059];
cameraUpVector = [-0.027 0.288 0.957];
cameraViewAngle = 0.33133;

params = YAML.read(params_config_file);

ground_removal_params = struct();
ground_removal_params.linefit_seg_r_min = params.linefit_seg_r_min;
ground_removal_params.linefit_seg_r_max = params.linefit_seg_r_max;
ground_removal_params.linefit_seg_n_bins = params.linefit_seg_n_bins;
ground_removal_params.linefit_seg_n_segments = params.linefit_seg_n_segments;
ground_removal_params.linefit_seg_max_dist_to_line = params.linefit_seg_max_dist_to_line;
ground_removal_params.linefit_seg_max_slope = params.linefit_seg_max_slope;
ground_removal_params.linefit_seg_max_fit_error = params.linefit_seg_max_fit_error;
ground_removal_params.linefit_seg_long_threshold = params.linefit_seg_long_threshold;
ground_removal_params.linefit_seg_max_long_height = params.linefit_seg_max_long_height;
ground_removal_params.linefit_seg_max_start_height = params.linefit_seg_max_start_height;
ground_removal_params.linefit_seg_sensor_height = params.linefit_seg_sensor_height;
ground_removal_params.linefit_seg_line_search_angle = params.linefit_seg_line_search_angle;
ground_removal_params.linefit_seg_n_threads = params.linefit_seg_n_threads;

ground_removal_params.ground_removal_method = params.groundRemoveMethod;

data_pointcloud_dir = [data_root '/' pointcloud_name];
pointCloudBase_file_name = sprintf('%s/%06d.pcd', data_pointcloud_dir, start_index);
pointCloudBase = pcread(pointCloudBase_file_name);
points = [pointCloudBase.Location pointCloudBase.Intensity];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
linefitGroundSegmentParams = struct();
linefitGroundSegmentParams.r_min_square = double(ground_removal_params.linefit_seg_r_min * ground_removal_params.linefit_seg_r_min);
linefitGroundSegmentParams.r_max_square = double(ground_removal_params.linefit_seg_r_max * ground_removal_params.linefit_seg_r_max);
linefitGroundSegmentParams.n_bins = double(ground_removal_params.linefit_seg_n_bins);
linefitGroundSegmentParams.n_segments = double(ground_removal_params.linefit_seg_n_segments);
linefitGroundSegmentParams.max_dist_to_line = double(ground_removal_params.linefit_seg_max_dist_to_line);
linefitGroundSegmentParams.max_slope = double(ground_removal_params.linefit_seg_max_slope);
linefitGroundSegmentParams.max_error_square = double(ground_removal_params.linefit_seg_max_fit_error * ground_removal_params.linefit_seg_max_fit_error);
linefitGroundSegmentParams.long_threshold = double(ground_removal_params.linefit_seg_long_threshold);
linefitGroundSegmentParams.max_long_height = double(ground_removal_params.linefit_seg_max_long_height);
linefitGroundSegmentParams.max_start_height = double(ground_removal_params.linefit_seg_max_start_height);
linefitGroundSegmentParams.sensor_height = double(ground_removal_params.linefit_seg_sensor_height);
linefitGroundSegmentParams.line_search_angle = double(ground_removal_params.linefit_seg_line_search_angle);
linefitGroundSegmentParams.n_threads = double(ground_removal_params.linefit_seg_n_threads);
linefitGroundSegmentParams.leveling = true;
linefitGroundSegmentParams.levelingPreset = true;
linefitGroundSegmentParams.levelingPresetZ = 0;
linefitGroundSegmentParams.levelingPresetPitch = 0;
linefitGroundSegmentParams.levelingPresetRoll = 0;

% [meter, degree, degree]
linefitGroundSegmentParams.leveling = false;
[ground_point_cloud, points_without_ground_stage1] = linefit_ground_segment(linefitGroundSegmentParams, double(points));
[mount_z, mount_pitch, mount_roll] = ransac_ground_estimation(ground_point_cloud);
linefitGroundSegmentParams.levelingPresetZ = mount_z;
linefitGroundSegmentParams.levelingPresetPitch = mount_pitch/180*pi;
linefitGroundSegmentParams.levelingPresetRoll = mount_roll/180*pi;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% show the estimated ground points without leveling
figure();
pcshow(pointCloud(points_without_ground_stage1(:, 1:3)), 'MarkerSize', 20);
set(gca, 'XTick', '');
set(gca, 'YTick', '');
set(gca, 'ZTick', '');
set(gca, 'XGrid', 'off');
set(gca, 'YGrid', 'off');
set(gca, 'ZGrid', 'off');
set(gca, 'XColor', [1 1 1]);
set(gca, 'YColor', [1 1 1]);
set(gca, 'ZColor', [1 1 1]);
set(gcf, 'Color', [1 1 1]);

set(gca, 'CameraPosition', cameraPosition);
set(gca, 'CameraTarget', cameraTarget);
set(gca, 'CameraUpVector', cameraUpVector);
set(gca, 'CameraViewAngle', cameraViewAngle);

linefitGroundSegmentParams.leveling = true;
[ground_point_cloud_estimated, points_without_ground_stage2] = linefit_ground_segment(linefitGroundSegmentParams, double(points));
[mount_z, mount_pitch, mount_roll] = ransac_ground_estimation(ground_point_cloud_estimated);

figure();
pcshow(pointCloud(points_without_ground_stage2(:, 1:3)), 'MarkerSize', 20);
set(gca, 'XTick', '');
set(gca, 'YTick', '');
set(gca, 'ZTick', '');
set(gca, 'XGrid', 'off');
set(gca, 'YGrid', 'off');
set(gca, 'ZGrid', 'off');
set(gca, 'XColor', [1 1 1]);
set(gca, 'YColor', [1 1 1]);
set(gca, 'ZColor', [1 1 1]);
set(gcf, 'Color', [1 1 1]);

set(gca, 'CameraPosition', cameraPosition);
set(gca, 'CameraTarget', cameraTarget);
set(gca, 'CameraUpVector', cameraUpVector);
set(gca, 'CameraViewAngle', cameraViewAngle);

fprintf('estimated mount_z: %f, mount_pitch: %f, mount_roll: %f\n', mount_z, mount_pitch, mount_roll);

lidar2ground_extrinsic_filename = sprintf('%s/lidar2ground_extrinsic.yml', data_root);

exportAutowareStyleYmlExtrinsic(lidar2ground_extrinsic_filename, mount_z, mount_pitch, mount_roll);

function exportAutowareStyleYmlExtrinsic(filename, mount_z, mount_pitch, mount_roll)
  % export the parameters to the yaml file
  fid = fopen(filename, 'w');
  LidarToGroundExtrinsicMat = eye(4);
  LidarToGroundExtrinsic_rot = eul2rotm([0 mount_pitch/180*pi mount_roll/180*pi]);
  LidarToGroundExtrinsicMat(1:3, 1:3) = LidarToGroundExtrinsic_rot;
  LidarToGroundExtrinsicMat(3, 4) = mount_z;
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  str = sprintf('LidarToGroundExtrinsicMat:\r\n');
  fprintf(fid, str);
  str = sprintf('  rows: %d\r\n', size(LidarToGroundExtrinsicMat, 1));
  fprintf(fid, str);
  str = sprintf('  cols: %d\r\n', size(LidarToGroundExtrinsicMat, 2));
  fprintf(fid, str);
  str = sprintf('  dt: d\r\n');
  fprintf(fid, str);
  str = sprintf('  data: ');
  fprintf(fid, str);
  str = sprintf('[ ');
  for i = 1 : size(LidarToGroundExtrinsicMat, 1)
      for j = 1 : size(LidarToGroundExtrinsicMat, 2)
          str = sprintf('%s%.6f', str, LidarToGroundExtrinsicMat(i, j));
          if ~(j == size(LidarToGroundExtrinsicMat, 2) && i == size(LidarToGroundExtrinsicMat, 1))
              str = sprintf('%s, ', str);
          end
      end
      if i ~= size(LidarToGroundExtrinsicMat, 1)
          str = sprintf('%s\r\n          ', str);
      else
          str = sprintf('%s]\r\n', str);
      end
  end
  fprintf(fid, str);
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  str = sprintf('mount_yaw: %.6f\r\n', 0);
  fprintf(fid, str);
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  str = sprintf('mount_pitch: %.6f\r\n', mount_pitch);
  fprintf(fid, str);
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  str = sprintf('mount_roll: %.6f\r\n', mount_roll);
  fprintf(fid, str);
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  str = sprintf('mount_x: %.6f\r\n', 0);
  fprintf(fid, str);
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  str = sprintf('mount_y: %.6f\r\n', 0);
  fprintf(fid, str);
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  str = sprintf('mount_z: %.6f\r\n', mount_z);
  fprintf(fid, str);
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  fclose(fid);

  warningCalibrationFileDoneDialog();

  function warningCalibrationFileDoneDialog
    str = sprintf('Export Calibration File Done! \r\n');
    uiwait(warndlg(str, 'Export Calibration File Done'));
  end % warningCalibrationFileDoneDialog
end




