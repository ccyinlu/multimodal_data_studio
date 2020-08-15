% generate the vetical angle of the pointCloud

clc;
close all;

addpath('../thirdParty/yaml');
addpath('../utils/interface');
addpath('../utils/cloudSegmentation/func');
addpath('../utils/cloudSegmentation/mex');

if ~exist('data_root')
    data_root = '/media/bingo/SSD/camera_lidar_calibration/triple/20200813';
end

if ~exist('pointcloud_name')
    pointcloud_name = 'lidar2';
end

start_index = 1;

params_config_file = [data_root '/' 'camera_lidar_calibration_config.yml'];
if ~exist(params_config_file,'file')
    return;
end

params = YAML.read(params_config_file);
pointcloud_limit = str2num(params.pointcloud_limit);
vertical_theta = str2num(params.vertical_theta) * pi / 180; % must be in radial
N_SCAN = params.N_SCAN;
Horizon_SCAN = params.Horizon_SCAN;

if ~exist('pointType')
    % pointType = 'hesai_p40p';
    pointType = 'prescan_p40p';
end

data_pointcloud_dir = [data_root '/' pointcloud_name];

pointCloudBase_file_name = sprintf('%s/%06d.pcd', data_pointcloud_dir, start_index);

pointCloudBase = pcread(pointCloudBase_file_name);

params = struct();
params.N_SCAN = N_SCAN;
params.Horizon_SCAN = Horizon_SCAN;
params.vertical_theta = vertical_theta;
points = [pointCloudBase.Location pointCloudBase.Intensity];

% you should convert the points from single to double, if you use the pcread, then the point type will be single
[~, ~, ~, v_angle_raw] = projectPointCloudMex(params, double(points));

v_angle_raw_deg = v_angle_raw * 180 / pi;
 
% kmeans
v_angle_raw_deg_id = kmeans(v_angle_raw_deg, params.N_SCAN);

v_angle_config = zeros(params.N_SCAN, 1);

for i = 1 : params.N_SCAN
    v_angle_raw_deg_sel = v_angle_raw_deg(v_angle_raw_deg_id == i);
    v_angle_config(i) = mean(v_angle_raw_deg_sel);
end

v_angle_config_descend = sort(v_angle_config, 'descend');
