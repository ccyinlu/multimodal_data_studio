function [mount_z, mount_pitch, mount_roll] = ransac_ground_estimation(groundSegmentParams, input_point_cloud)
  % 
  groundSegmentParams.leveling = false;
  [ground_point_cloud, ~] = linefit_ground_segment(groundSegmentParams, input_point_cloud);
  % estimate the plane equation according to RANSAC method

  ground_point_cloud_pt = pointCloud(ground_point_cloud);

  planeFit_maxDistance = 0.05;
  planeFit_referenceVector = [0 0 1];
  planeFit_maxAngularDistance = 20; % degree
  ground_plane_model = pcfitplane(ground_point_cloud_pt, planeFit_maxDistance, planeFit_referenceVector, planeFit_maxAngularDistance);

  childNormals = ground_plane_model.Normal;
  parentNormals = [0 0 1];
  initExtrinsicParamsQ = eul2quat([0 0 0]);
  extrinsicParamsQ = estimatePitchRollByCoNormalCeresMex(double(parentNormals), ...
                                                        double(childNormals), ...
                                                        double(initExtrinsicParamsQ), ...
                                                        false);

  extrinsicParamsEuler = quat2eul([
    extrinsicParamsQ(1) ...
    extrinsicParamsQ(2) ...
    extrinsicParamsQ(3) ...
    extrinsicParamsQ(4) ...
  ]); % [yaw pitch roll]

  mount_z = ground_plane_model.Parameters(4);
  mount_pitch = extrinsicParamsEuler(2) * 180 / pi; % degree
  mount_roll = extrinsicParamsEuler(3) * 180 / pi; % degree
end