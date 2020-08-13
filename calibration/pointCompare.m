function [pcObjectPointOfUnmatched, pcObjectLimited] = pointCompare(pcBase, pcObject, pointCompareParams, debug_)
    % return the index of the cluster = pcObject - pcBase

    limits = pointCompareParams.limits;
    ifRemoveGround = pointCompareParams.ifRemoveGround;

    ground_removal_params = struct();
    ground_removal_params.vertical_theta = pointCompareParams.vertical_theta;
    ground_removal_params.N_SCAN = pointCompareParams.N_SCAN;
    ground_removal_params.Horizon_SCAN = pointCompareParams.Horizon_SCAN;
    ground_removal_params.groundScanInd = pointCompareParams.groundScanInd;
    ground_removal_params.sensorMountAngle = pointCompareParams.sensorMountAngle;
    ground_removal_params.groundRemovalAngleT = pointCompareParams.groundRemovalAngleT;


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
        pcBasePoint = groundRemove(pcBasePoint, ground_removal_params);
        pcObjectPoint = groundRemove(pcObjectPoint, ground_removal_params);
    end

    % we just use location for unmatching
    % pcBasePoint = pcBasePoint(:, 1:3);
    % pcObjectPoint = pcObjectPoint(:, 1:3);

    MdlKDT = KDTreeSearcher(pcBasePoint(:, 1:3));
    [inxKDT, distance] = knnsearch(MdlKDT, pcObjectPoint(:, 1:3));

    % calc the std of the distance and select the unmatched point
    indexOfUnmatched = find(distance > 3 * std(distance, 'omitnan'));

    pcObjectPointOfUnmatched = pcObjectPoint(indexOfUnmatched, :);

    if debug_
        fig = figure();
        set(fig,'position',[100, 130, 1280, 720]);
        current_axes = axes('position',[0,0,1,1]);
        
        cla(current_axes); 
        pointsShow(current_axes, ...
                [pcObjectLimited.Location pcObjectLimited.Intensity], ...
                limits, ...
                [0 255], ...
                'height', ...
                3);
        hold(current_axes, 'on');
        scatter3(current_axes, ...
                pcObjectPointOfUnmatched(:, 1), ...
                pcObjectPointOfUnmatched(:, 2), ...
                pcObjectPointOfUnmatched(:, 3), ...
                3 * 1.5, ...
                'r', 'filled');
        axis(current_axes, 'equal');
        % scatter3(pcObjectPointOfUnmatched(:, 1), pcObjectPointOfUnmatched(:, 2), pcObjectPointOfUnmatched(:, 3), 3, 'filled');
    end
end