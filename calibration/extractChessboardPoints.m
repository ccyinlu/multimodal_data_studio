function [isFound, chessboardPoints, normal] = extractChessboardPoints(pcBase, ...
                                                                       pcObject, ...
                                                                       params)
    
    pointcloud_limit = params.pointcloud_limit; % [ -50 50 -100 0 -3 3]; % [x_min x_max y_min y_max z_min z_max]
    planeFitParams = params.planeFitParams; % {0.02 [0 1 0] 70}; % maxDistance, reference normal, maxAngular
    planeFitParams_stage_1 = params.planeFitParams_stage_1; % {0.08 [0 1 0] 70}; % maxDistance, reference normal, maxAngular
    planeFitParams_stage_2 = params.planeFitParams_stage_2; % {0.02 [0 1 0] 70}; % maxDistance, reference normal, maxAngular
    distanceMat_T = params.distanceMat_T; % 0.05;

    pointCompareParams = struct();
    pointCompareParams.limits = params.pointcloud_limit;
    pointCompareParams.ifRemoveGround = params.ifRemoveGround;
    pointCompareParams.vertical_theta = params.vertical_theta;
    pointCompareParams.N_SCAN = params.N_SCAN;
    pointCompareParams.Horizon_SCAN = params.Horizon_SCAN;
    pointCompareParams.groundScanInd = params.groundScanInd;
    pointCompareParams.sensorMountAngle = params.sensorMountAngle;
    pointCompareParams.groundRemovalAngleT = params.groundRemovalAngleT;

    cloudSegmentationParams = struct();
    cloudSegmentationParams.vertical_theta = params.vertical_theta;
    cloudSegmentationParams.N_SCAN = params.N_SCAN;
    cloudSegmentationParams.Horizon_SCAN = params.Horizon_SCAN;
    cloudSegmentationParams.groundScanInd = params.groundScanInd;
    cloudSegmentationParams.sensorMountAngle = params.sensorMountAngle;
    cloudSegmentationParams.groundRemovalAngleT = params.groundRemovalAngleT;
    cloudSegmentationParams.segmentTheta = params.segmentTheta;
    cloudSegmentationParams.feasibleSegmentValidPointNum = params.feasibleSegmentValidPointNum;
    cloudSegmentationParams.segmentValidPointNum = params.segmentValidPointNum;
    cloudSegmentationParams.segmentValidLineNum = params.segmentValidLineNum;

    distanceMatSegmentationParams = struct();
    distanceMatSegmentationParams.horizontal_res = params.horizontal_res;
    distanceMatSegmentationParams.vertical_theta = params.vertical_theta;

    [pcObjectPointOfUnmatched, ~] = pointCompare(pcBase, pcObject, pointCompareParams, false);

    isFound = 0;
    chessboardPoints = [];
    normal = [];

    % select the difference pointcloud
    if isempty(pcObjectPointOfUnmatched)
        pointcloudUsed = 0;
        plane_model = [];
        chessboard_points_estimated = [];
        chessboard_points_estimated_refined = [];
        fprintf('not unmatched points found\n');
        return;
    end

    % first cluster the pointcloud
    pointCloudSegmentedCluster_origin = cloudSegmentation(pcObjectPointOfUnmatched, cloudSegmentationParams);
    pointCloudSegmentedCluster_origin_num = zeros(size(pointCloudSegmentedCluster_origin));
    % remove the background cluster equals to zeros 
    for i = 1:length(pointCloudSegmentedCluster_origin)
        pointCloudSegmentedCluster_origin_num(i) = size(pointCloudSegmentedCluster_origin{i}, 1);
    end
    [~, maxIndex] = max(pointCloudSegmentedCluster_origin_num);

    pointCloudSegmentedCluster = [];
    pointCloudSegmentedCluster_count = 0;
    for i = 1:length(pointCloudSegmentedCluster_origin)
        if i == maxIndex
            continue;
        end
        pointCloudSegmentedCluster_count = pointCloudSegmentedCluster_count + 1;
        pointCloudSegmentedCluster{pointCloudSegmentedCluster_count} = pointCloudSegmentedCluster_origin{i};
    end

    if isempty(pointCloudSegmentedCluster)
        pointcloudUsed = 0;
        plane_model = [];
        chessboard_points_estimated = [];
        chessboard_points_estimated_refined = [];
        return;
    end

    % circle the pointCloudSegmentationCluster, judge the cluster if a significant plane exist, then choose the most significant plane
    significantScores = zeros(length(pointCloudSegmentedCluster), 1);

    for i = 1 : length(pointCloudSegmentedCluster)
        current_pointCloudSegmentedCluster = pointCloudSegmentedCluster{i};
        current_pointCloudSegmentedCluster = pointCloud(current_pointCloudSegmentedCluster(:, 1:3));
        % find planes of the unmatched pointcloud
        [model, inlierIndices, outlierIndices] = pcfitplane(current_pointCloudSegmentedCluster, ...
                                        planeFitParams{1}, ...
                                        planeFitParams{2}, ...
                                        planeFitParams{3});
        if length(inlierIndices) < 30
            significantScores(i) = 0;
            continue;
        end

        significantScores(i) = 0.3 * length(inlierIndices)/( length(inlierIndices) + length(outlierIndices) ) + 0.7 * size(current_pointCloudSegmentedCluster.Location, 1)/size(pcObjectPointOfUnmatched, 1);
    end

    % pick the highest significantScore
    [~, maxIndex] = max(significantScores);
    
    if(significantScores(maxIndex) < 0.0)
        pointcloudUsed = 0;
        plane_model = [];
        chessboard_points_estimated = [];
        chessboard_points_estimated_refined = [];
        return;
    else
        pointcloudUsed = 1;
        pointCloudSegmentedCluster_significant = pointCloudSegmentedCluster{maxIndex};
        chessboardPoints = pointCloud(pointCloudSegmentedCluster_significant(:, 1:3));

        % find planes of the unmatched pointcloud
        [plane_model_first, plane_inlierIndices_first, plane_outlierIndices_first] = pcfitplane(pointCloud(pointCloudSegmentedCluster_significant(:, 1:3)), ...
                                                                                                planeFitParams_stage_1{1}, ...
                                                                                                planeFitParams_stage_1{2}, ...
                                                                                                planeFitParams_stage_1{3}, ...
                                                                                                'MaxNumTrials', 10000);

        plane_points_candidates = pointCloudSegmentedCluster_significant(plane_inlierIndices_first, 1:3);
        plane_model = plane_model_first;

        %% generate the distanceMat
        distanceMatFull = distanceMatSegmentation(pointCloudSegmentedCluster_significant, plane_model, distanceMatSegmentationParams);
        % generate the threshold for the distanceMat automatically
        % distanceMat = distanceMatFull(:,:, 4);
        % distanceMat_BW = imbinarize(distanceMat);
        % distanceMat_segmented = distanceMat;
        % distanceMat_segmented(distanceMat_BW == 1) = 0.5;
        % distanceMat_segmented(~isnan(distanceMat) & distanceMat_BW == 0) = 0;
        % distanceMat_segmented(isnan(distanceMat)) = 1;

        distanceMat = distanceMatFull(:,:, 4);
        distanceMat_segmented = distanceMat;
        distanceMat_segmented(distanceMat_segmented > distanceMat_T) = 0.5;
        distanceMat_segmented(distanceMat_segmented <= distanceMat_T) = 0.0;
        distanceMat_segmented(isnan(distanceMat_segmented)) = 1;

        %% parse the region and get the maxest connected-region
        % search the connected region
        [L, n] = bwlabel(distanceMat_segmented == 0);
        region_max = 0;
        region_max_index = 0;
        for i  = 1 : n
            status = regionprops(L == i);
            % filter the mask via area
            if status.Area > region_max
                region_max = status.Area;
                region_max_index = i;
            end
        end
        distanceMat_chessboard_segmented = L == region_max_index;
        % get the raw 3D points according to the chessboard
        distanceMatFull_x = distanceMatFull(:, :, 1);
        distanceMatFull_y = distanceMatFull(:, :, 2);
        distanceMatFull_z = distanceMatFull(:, :, 3);
        distanceMatFull_i = distanceMatFull(:, :, 6);
        chessboard_points_estimated_x = distanceMatFull_x(distanceMat_chessboard_segmented == 1);
        chessboard_points_estimated_y = distanceMatFull_y(distanceMat_chessboard_segmented == 1);
        chessboard_points_estimated_z = distanceMatFull_z(distanceMat_chessboard_segmented == 1);
        chessboard_points_estimated_i = distanceMatFull_i(distanceMat_chessboard_segmented == 1);
        chessboard_points_estimated = [chessboard_points_estimated_x chessboard_points_estimated_y chessboard_points_estimated_z chessboard_points_estimated_i];

        %% refine the plane parameters and re-extract the chessboard points
        [plane_model_refined, plane_inlierIndices_refined, plane_outlierIndices_first] = pcfitplane(pointCloud(chessboard_points_estimated(:, 1:3)), ...
                                                                                                    planeFitParams_stage_2{1}, ...
                                                                                                    planeFitParams_stage_2{2}, ...
                                                                                                    planeFitParams_stage_2{3}, ...
                                                                                                    'MaxNumTrials', 10000);

        distanceMatFull_refined = distanceMatSegmentation(pointCloudSegmentedCluster_significant, plane_model_refined, distanceMatSegmentationParams);

        % distanceMat_refined = distanceMatFull_refined(:,:, 4);
        % distanceMat_BW_refined = imbinarize(distanceMat_refined);
        % distanceMat_segmented_refined = distanceMat_refined;
        % distanceMat_segmented_refined(distanceMat_BW_refined == 1) = 0.5;
        % distanceMat_segmented_refined(~isnan(distanceMat) & distanceMat_BW_refined == 0) = 0;
        % distanceMat_segmented_refined(isnan(distanceMat)) = 1;

        distanceMat_refined = distanceMatFull_refined(:,:, 4);
        distanceMat_segmented_refined = distanceMat_refined;
        distanceMat_segmented_refined(distanceMat_segmented_refined > distanceMat_T) = 0.5;
        distanceMat_segmented_refined(distanceMat_segmented_refined <= distanceMat_T) = 0.0;
        distanceMat_segmented_refined(isnan(distanceMat_segmented)) = 1;

        %% parse the region and get the maxest connected-region
        % search the connected region
        [L, n] = bwlabel(distanceMat_segmented_refined == 0);
        region_max = 0;
        region_max_index = 0;
        for i  = 1 : n
            status = regionprops(L == i);
            % filter the mask via area
            if status.Area > region_max
                region_max = status.Area;
                region_max_index = i;
            end
        end
        distanceMat_chessboard_segmented_refined = L == region_max_index;
        % get the raw 3D points according to the chessboard
        distanceMatFull_x = distanceMatFull_refined(:, :, 1);
        distanceMatFull_y = distanceMatFull_refined(:, :, 2);
        distanceMatFull_z = distanceMatFull_refined(:, :, 3);
        distanceMatFull_i = distanceMatFull_refined(:, :, 6);
        chessboard_points_estimated_x = distanceMatFull_x(distanceMat_chessboard_segmented_refined == 1);
        chessboard_points_estimated_y = distanceMatFull_y(distanceMat_chessboard_segmented_refined == 1);
        chessboard_points_estimated_z = distanceMatFull_z(distanceMat_chessboard_segmented_refined == 1);
        chessboard_points_estimated_i = distanceMatFull_i(distanceMat_chessboard_segmented_refined == 1);
        chessboard_points_estimated_refined = [chessboard_points_estimated_x chessboard_points_estimated_y chessboard_points_estimated_z chessboard_points_estimated_i];

    end

    isFound = 1;
    chessboardPoints = chessboard_points_estimated_refined;
    normal = plane_model.Normal;
end