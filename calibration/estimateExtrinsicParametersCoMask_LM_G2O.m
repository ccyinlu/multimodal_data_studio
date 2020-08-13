function [Params, estimationErrors, varargout] = estimateExtrinsicParametersCoMask_LM_G2O(  chessboardLidarPoints, ...
                                                                                            chessboardPlaneMaskDT, ...
                                                                                            cameraParams, ...
                                                                                            estimateExtrinsicParams)
    % estimate the extrinsic parameters according to the chessboard points and mask using the LM method
    % input:
    % chessboardLidarPoints: Mx1 cell, each cell, NX3 points
    % chessboardPlaneMaskDT: Mx1 cell, each cell, HXW singles
    % cameraParams: intrinsic params
    % estimateExtrinsicParams: struct with fields initExtrinsicParams, verbose, maxIters
    warning('off','all');

    global ifVerbose;

    Params = struct();
    Params.opt = [];
    Params.extrinsicMat = eye(4);
    estimationErrors = [];

    K = cameraParams.IntrinsicMatrix';

    imageSize = [cameraParams.ImageSize(2) cameraParams.ImageSize(1)];

    initExtrinsicParams = estimateExtrinsicParams.initExtrinsicParams;
    ifVerbose = estimateExtrinsicParams.verbose;
    maxIters = estimateExtrinsicParams.maxIters;

    % parse the Distortion parameter
    if length(cameraParams.RadialDistortion) == 2
        D = [cameraParams.RadialDistortion 0 cameraParams.TangentialDistortion];
    elseif length(cameraParams.RadialDistortion) == 3
        D = [cameraParams.RadialDistortion cameraParams.TangentialDistortion];
    end

    % xopt equals lidar2camera se3
    % chessboardLidarPoints should be Mx1 single cell
    % chessboardPlaneMaskDT should be Mx1 single cell
    chessboardLidarPoints_ = cell(length(chessboardLidarPoints), 1);
    chessboardPlaneMaskDT_ = cell(length(chessboardPlaneMaskDT), 1);
    for i = 1 : length(chessboardLidarPoints_)
        chessboardLidarPoints_{i} = single(chessboardLidarPoints{i});
    end
    for i = 1 : length(chessboardPlaneMaskDT_)
        chessboardPlaneMaskDT_{i} = single(chessboardPlaneMaskDT{i});
    end
    xopt = EdgeSE3ProjectDirectWithDirstortG2oLMMex(double(initExtrinsicParams), ...
                                                    chessboardLidarPoints_, ...
                                                    chessboardPlaneMaskDT_, ...
                                                    double(D), ...
                                                    double(K), ...
                                                    double(imageSize), ...
                                                    double(ifVerbose), ...
                                                    double(maxIters));

    Params.opt = eulerInv(se32euler(xopt));

    Params.extrinsicMat = euler2matrix(Params.opt);

    planeMaskMatchingEnergyLoss(eulerInv(se32euler(initExtrinsicParams)), ...
                                chessboardLidarPoints_, ...
                                chessboardPlaneMaskDT_, ...
                                D, ...
                                K, ...
                                imageSize);
    estimationErrors = planeMaskMatchingEnergyLoss(Params.opt, ...
                                                    chessboardLidarPoints_, ...
                                                    chessboardPlaneMaskDT_, ...
                                                    D, ...
                                                    K, ...
                                                    imageSize);

    varargout{1} = 0; % fmin
    varargout{2} = 0; % k
    varargout{3} = 0; % mks
    varargout{4} = 0; %opti_in_process

function val = planeMaskMatchingEnergyLoss(x, chessboardLidarPoints, chessboardPlaneMaskDT, D, K, imageSize)
    k1 = 0;
    k2 = 0;
    k3 = 0;
    p1 = 0;
    p2 = 0;
    if length(D) == 2
        k1 = D(1);
        k2 = D(2);
    elseif length(D) == 3
        k1 = D(1);
        k2 = D(2);
        k3 = D(3);
    elseif length(D) == 4
        k1 = D(1);
        k2 = D(2);
        p1 = D(3);
        p2 = D(4);
    elseif length(D) == 5
        k1 = D(1);
        k2 = D(2);
        k3 = D(3);
        p1 = D(4);
        p2 = D(5);
    end

    % parse the K
    fx = K(1, 1);
    fy = K(2, 2);
    cx = K(1, 3);
    cy = K(2, 3);

    % get the rotation matrix from the euler angles
    rmat = eul2rotm([x(1) x(2) x(3)]);
    vmat = [x(4) x(5) x(6)];

    R_l2c = rmat';
    T_l2c = -R_l2c * vmat';
    Trans_lidar_2_cam = eye(4);
    Trans_lidar_2_cam(1:3, 1:3) = R_l2c;
    Trans_lidar_2_cam(1:3, 4) = T_l2c;

    pairNum = length(chessboardLidarPoints);
    object_count = 0;
    object_error = 0;
    count = 0;

    for i = 1 : pairNum

        mask_D = chessboardPlaneMaskDT{i};
        points = chessboardLidarPoints{i};

        % get the range of the chessboard lidar center
        points_center = mean(points, 1);
        points_range = norm(points_center);
        points_weight = points_range;

        if isempty(points)
            continue;
        end

        points(:, 4) = ones(size(points, 1), 1); % nx4

        Pc = Trans_lidar_2_cam*points'; %4xn

        valid_index = Pc(3,:) > 1;
        Pc = Pc(:, valid_index);

        tempX = Pc(1,:)./ Pc(3,:);
        tempY = Pc(2,:)./ Pc(3,:);

        r2 = tempX.^2 + tempY.^2;
        tempXX = (1 + k1*r2 + k2*r2.^2 + k3*r2.^3).*tempX + 2 * p1 * tempX.*tempY + p2 * (r2 + 2*tempX.^2);
        tempYY = (1 + k1*r2 + k2*r2.^2 + k3*r2.^3).*tempY + p1 * (r2 + 2*tempY.^2) + 2 * p2 * tempX.*tempY;

        tempXX = tempXX * fx + cx;
        tempYY = tempYY * fy + cy;

        index = tempXX(1,:) > 1 & tempXX(1,:) < imageSize(1) & tempYY(1,:) > 1 & tempYY(1,:) < imageSize(2); 
        tempX_final = tempXX(index);
        tempY_final = tempYY(index);

        count = length(tempXX);
        error_ = sum(mask_D(sub2ind([imageSize(2), imageSize(1)], floor(tempY_final), floor(tempX_final))));
        error_ = error_ + 10000 * sum(~index);

        object_error = object_error + error_/count;
        object_count = object_count + 1;
    end

    val = object_error/object_count;
    global ifVerbose;
    if ifVerbose
        fprintf('LM_optimize eval #%d: %.6f, [%.4f %.4f %.4f %.4f %.4f %.4f]\n', 1, ...
                                                                                val, ...
                                                                                x(1), ...
                                                                                x(2), ...
                                                                                x(3), ...
                                                                                x(4), ...
                                                                                x(5), ...
                                                                                x(6));
end
