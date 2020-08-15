function points_ground_filtered = ground_filter_by_plane(points, planeModel, distanceMat_T)
  % calc the distance between the points and the plane
  plane_func_params = planeModel.Parameters;
  plane_normal_norm = norm(plane_func_params(1:3));

  points_extend = [points(:, 1:3) ones(size(points, 1), 1)];
  distance = abs(points_extend * plane_func_params') / plane_normal_norm;

  points_ground_filtered = points(distance > distanceMat_T, :);
end