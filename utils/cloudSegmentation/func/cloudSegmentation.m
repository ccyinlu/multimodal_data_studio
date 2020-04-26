function pointCloudSegmentedCluster = cloudSegmentation(pointsFull, cloudSegmentationParams)
    % pointsFull: Mx3 or  points
    params = struct();
    points = pointsFull;

    vertical_theta = cloudSegmentationParams.vertical_theta;
    vertical_theta = vertical_theta/180*pi;
    params.N_SCAN = cloudSegmentationParams.N_SCAN;
    params.Horizon_SCAN = cloudSegmentationParams.Horizon_SCAN;
    params.vertical_theta = vertical_theta;

    params.groundScanInd = cloudSegmentationParams.groundScanInd;
    params.sensorMountAngle = cloudSegmentationParams.sensorMountAngle;
    params.groundRemovalAngleT = cloudSegmentationParams.groundRemovalAngleT;
    params.segmentTheta = cloudSegmentationParams.segmentTheta;
    params.feasibleSegmentValidPointNum = cloudSegmentationParams.feasibleSegmentValidPointNum;
    params.segmentValidPointNum = cloudSegmentationParams.segmentValidPointNum;
    params.segmentValidLineNum = cloudSegmentationParams.segmentValidLineNum;
    params.debugInfo = [18 7 14];

    % you should convert the points from single to double, if you use the pcread, then the point type will be single
    [~, rangeMatFilled, ~, ~] = projectPointCloudMex(params, double(points));

    [labelMatFilled] = cloudSegmentationMex(params, double(rangeMatFilled), int8(zeros(size(rangeMatFilled))));

    labels = unique(labelMatFilled);
    cluster_num = length(labels);
    result = reshape(rangeMatFilled(:, :, 3:6), [], 4);
    pointCloudSegmentedCluster_num = 0;
    pointCloudSegmentedCluster = [];
    for i = 1 : cluster_num
        if labels(i) == 999999
            continue;
        end
        pointCloudSegmentedCluster_num = pointCloudSegmentedCluster_num + 1;
        label_index = find(labelMatFilled == labels(i));
        pointCloudSegmentedCluster{pointCloudSegmentedCluster_num} = result(label_index, :);
    end
end