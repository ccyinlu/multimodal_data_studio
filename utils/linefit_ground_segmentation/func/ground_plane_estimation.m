function plane_equation = ground_plane_estimation(groundSegmentParams, input_point_cloud)
  % estimate the partial ground points
  groundSegmentParams.leveling = false;
  [ground_point_cloud, ~] = linefit_ground_segment(groundSegmentParams, input_point_cloud);
  % estimate the plane equation according to RANSAC method

  ground_point_cloud_pt = pointCloud(ground_point_cloud);

  planeFit_maxDistance = 0.05;
  planeFit_referenceVector = [0 0 1];
  planeFit_maxAngularDistance = 20; % degree
  ground_plane_model = pcfitplane(ground_point_cloud_pt, planeFit_maxDistance, planeFit_referenceVector, planeFit_maxAngularDistance);

  plane_equation = ground_plane_model.Parameters;
end