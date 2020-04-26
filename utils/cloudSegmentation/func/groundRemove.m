function pointsGroundRemoved = groundRemove(pointsFull, ground_removal_params)
    % pointsFull: Mx3 or  points
    params = struct();
    points = pointsFull;

    vertical_theta = ground_removal_params.vertical_theta;
    vertical_theta = vertical_theta/180*pi;

    params.N_SCAN = ground_removal_params.N_SCAN;
    params.Horizon_SCAN = ground_removal_params.Horizon_SCAN;
    params.vertical_theta = vertical_theta;

    params.groundScanInd = ground_removal_params.groundScanInd;
    params.sensorMountAngle = ground_removal_params.sensorMountAngle;
    params.groundRemovalAngleT = ground_removal_params.groundRemovalAngleT;

    % you should convert the points from single to double, if you use the pcread, then the point type will be single
    [~, rangeMatFilled, ~, ~] = projectPointCloudMex(params, double(points));
    groundLabelFilled = groundRemovalMex(params, double(rangeMatFilled));
    no_ground_index = find(groundLabelFilled ~= 1);
    result = reshape(rangeMatFilled(:, :, 3:6), [], 4);
    pointsGroundRemoved = result(no_ground_index, :);
end