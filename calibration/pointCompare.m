function [pcObjectPointOfUnmatched, pcObjectLimited] = pointCompare(pcBase, pcObject, pointCompareParams)
    % return the index of the cluster = pcObject - pcBase

    limits = pointCompareParams.limits;
    ifRemoveGround = pointCompareParams.ifRemoveGround;
    ground_removal_method = pointCompareParams.groundRemoveMethod;

    ground_removal_params = struct();
    ground_removal_params.vertical_theta = pointCompareParams.vertical_theta;
    ground_removal_params.N_SCAN = pointCompareParams.N_SCAN;
    ground_removal_params.Horizon_SCAN = pointCompareParams.Horizon_SCAN;
    ground_removal_params.groundScanInd = pointCompareParams.groundScanInd;
    ground_removal_params.sensorMountAngle = pointCompareParams.sensorMountAngle;
    ground_removal_params.groundRemovalAngleT = pointCompareParams.groundRemovalAngleT;

    ground_removal_params.linefit_seg_r_min = pointCompareParams.linefit_seg_r_min;
    ground_removal_params.linefit_seg_r_max = pointCompareParams.linefit_seg_r_max;
    ground_removal_params.linefit_seg_n_bins = pointCompareParams.linefit_seg_n_bins;
    ground_removal_params.linefit_seg_n_segments = pointCompareParams.linefit_seg_n_segments;
    ground_removal_params.linefit_seg_max_dist_to_line = pointCompareParams.linefit_seg_max_dist_to_line;
    ground_removal_params.linefit_seg_max_slope = pointCompareParams.linefit_seg_max_slope;
    ground_removal_params.linefit_seg_max_fit_error = pointCompareParams.linefit_seg_max_fit_error;
    ground_removal_params.linefit_seg_long_threshold = pointCompareParams.linefit_seg_long_threshold;
    ground_removal_params.linefit_seg_max_long_height = pointCompareParams.linefit_seg_max_long_height;
    ground_removal_params.linefit_seg_max_start_height = pointCompareParams.linefit_seg_max_start_height;
    ground_removal_params.linefit_seg_sensor_height = pointCompareParams.linefit_seg_sensor_height;
    ground_removal_params.linefit_seg_line_search_angle = pointCompareParams.linefit_seg_line_search_angle;
    ground_removal_params.linefit_seg_n_threads = pointCompareParams.linefit_seg_n_threads;

    ground_removal_params.differPointRatio = pointCompareParams.differPointRatio;


    % select the ROI of the point according to the limits
    pcBasePoint = pcBase.Location;
    indexBaseValid = find(pcBasePoint(:, 1) > limits(1) & ...
                        pcBasePoint(:, 1) < limits(2) & ...
                        pcBasePoint(:, 2) > limits(3) & ...
                        pcBasePoint(:, 2) < limits(4) & ...
                        pcBasePoint(:, 3) > limits(5) & ...
                        pcBasePoint(:, 3) < limits(6));
    pcObjectPoint = pcObject.Location;
    indexObjectValid = find(pcObjectPoint(:, 1) > limits(1) & ...
                        pcObjectPoint(:, 1) < limits(2) & ...
                        pcObjectPoint(:, 2) > limits(3) & ...
                        pcObjectPoint(:, 2) < limits(4) & ...
                        pcObjectPoint(:, 3) > limits(5) & ...
                        pcObjectPoint(:, 3) < limits(6));

    pcBaseLimited = select(pcBase, indexBaseValid);
    pcObjectLimited = select(pcObject, indexObjectValid);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% you can not using the matching algos to eliminate the micro motion
    % due to the noise of the measurement, there exist mismatch beteen pcBase and pcObject
    % align the two pointclouds using icp
    % [~, pcObjectReg] = pcregrigid(pcObject, pcBase, 'MaxIterations', 50, 'Extrapolate', true, 'Tolerance', [0.0000001 0.0000009]);
    % if debug
    %     pcshowpair(pcBase, pcObjectReg);
    % end

    % find the correspondence point from the pcBase using knnsearch
    % define a default four-dimentional k d-tree
    if ~isempty(pcBaseLimited.Intensity) && ~isempty(pcBaseLimited.Intensity)
        pcBasePoint = [ pcBaseLimited.Location(:, 1) ...
                        pcBaseLimited.Location(:, 2) ...
                        pcBaseLimited.Location(:, 3) ...
                        pcBaseLimited.Intensity(:)];
        pcObjectPoint = [   pcObjectLimited.Location(:, 1) ...
                            pcObjectLimited.Location(:, 2) ...
                            pcObjectLimited.Location(:, 3) ...
                            pcObjectLimited.Intensity(:)];
    else
        pcBasePoint = [ pcBaseLimited.Location(:, 1) ...
                        pcBaseLimited.Location(:, 2) ...
                        pcBaseLimited.Location(:, 3)];
        pcObjectPoint = [   pcObjectLimited.Location(:, 1) ...
                            pcObjectLimited.Location(:, 2) ...
                            pcObjectLimited.Location(:, 3)];
    end

    % before the point compare, first remove the ground
    if ifRemoveGround
      if isequal(ground_removal_method, 'rangeMat')
        pcBasePoint = groundRemove(pcBasePoint, ground_removal_params);
        pcObjectPoint = groundRemove(pcObjectPoint, ground_removal_params);
      else if isequal(ground_removal_method, 'linefit')
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
        % 
        linefitGroundSegmentParams.leveling = false;
        [ground_point_cloud, ~] = linefit_ground_segment(linefitGroundSegmentParams, double(pcBasePoint));
        [mount_z, mount_pitch, mount_roll] = ransac_ground_estimation(ground_point_cloud);
        linefitGroundSegmentParams.levelingPresetZ = mount_z;
        linefitGroundSegmentParams.levelingPresetPitch = mount_pitch/180*pi;
        linefitGroundSegmentParams.levelingPresetRoll = mount_roll/180*pi;

        [~, pcBasePoint] = linefit_ground_segment(linefitGroundSegmentParams, double(pcBasePoint));
        [~, pcObjectPoint] = linefit_ground_segment(linefitGroundSegmentParams, double(pcObjectPoint));
      else
        errordlg('unsupport ground removal method', 'Error ground removal method');
      end
    end

    % we just use location for unmatching
    % pcBasePoint = pcBasePoint(:, 1:3);
    % pcObjectPoint = pcObjectPoint(:, 1:3);

    MdlKDT = KDTreeSearcher(pcBasePoint(:, 1:3));
    [inxKDT, distance] = knnsearch(MdlKDT, pcObjectPoint(:, 1:3));

    % calc the std of the distance and select the unmatched point
    indexOfUnmatched = find(distance > ground_removal_params.differPointRatio * std(distance, 'omitnan'));

    pcObjectPointOfUnmatched = pcObjectPoint(indexOfUnmatched, :);

end