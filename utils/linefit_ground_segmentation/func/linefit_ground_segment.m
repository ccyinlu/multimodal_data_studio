function [ground_point_cloud, obstacle_point_cloud] = linefit_ground_segment(groundSegmentParams, input_point_cloud)
  % judge if the [roll pitch z] parameters given, that is the leveling is set true

  input_point_cloud = input_point_cloud(:, 1:3);

  if ~isfield(groundSegmentParams, 'leveling')
    error('leveling parameter missing!');
  end

  if ~groundSegmentParams.leveling
    [ground_point_cloud, obstacle_point_cloud] = groundSegmentationMex(groundSegmentParams, input_point_cloud);
  else
    if ~isfield(groundSegmentParams, 'levelingPreset')
      error('levelingPreset parameter missing!');
    end

    if groundSegmentParams.levelingPreset
      % using the preset [z roll pitch] to transform the pointCloud
      if ~isfield(groundSegmentParams, 'levelingPresetZ') | ~isfield(groundSegmentParams, 'levelingPresetRoll') | ~isfield(groundSegmentParams, 'levelingPresetPitch')
        error('levelingPresetZ, levelingPresetRoll or levelingPresetPitch parameter missing!');
      end
      % construct the transformation matrix
      levelingMatrix = eye(4);
      levelingEulerMatrix = eul2rotm([0 groundSegmentParams.levelingPresetPitch groundSegmentParams.levelingPresetRoll]);
      levelingMatrix(1:3, 1:3) = levelingEulerMatrix;
      groundSegmentParams.sensor_height = groundSegmentParams.levelingPresetZ;

      % adjust the input_point_cloud
      input_point_cloud_hom = [input_point_cloud ones(size(input_point_cloud, 1), 1)]; % N x 4
      leveling_input_point_cloud_hom = (levelingMatrix * input_point_cloud_hom')'; % N x 4
      leveling_input_point_cloud = leveling_input_point_cloud_hom(:, 1:3);

      % applying the leveling_input_point_cloud
      [leveling_ground_point_cloud, leveling_obstacle_point_cloud] = groundSegmentationMex(groundSegmentParams, leveling_input_point_cloud);
      levelingMatrixInv = inv(levelingMatrix);

      leveling_ground_point_cloud_hom = [leveling_ground_point_cloud ones(size(leveling_ground_point_cloud, 1), 1)]; % M x 4
      ground_point_cloud_hom = (levelingMatrixInv * leveling_ground_point_cloud_hom')'; % M x 4
      ground_point_cloud = ground_point_cloud_hom(:, 1:3);

      leveling_obstacle_point_cloud_hom = [leveling_obstacle_point_cloud ones(size(leveling_obstacle_point_cloud, 1), 1)]; % P x 4
      obstacle_point_cloud_hom = (levelingMatrixInv * leveling_obstacle_point_cloud_hom')'; % P x 4
      obstacle_point_cloud = obstacle_point_cloud_hom(:, 1:3);
    end
  end
end