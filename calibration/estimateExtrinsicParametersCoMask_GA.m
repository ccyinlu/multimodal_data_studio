function [Params, estimationErrors, varargout] = estimateExtrinsicParametersCoMask_GA(  chessboardLidarPoints, ...
                                                                                    chessboardPlaneMaskDT, ...
                                                                                    cameraParams, ...
                                                                                    estimateExtrinsicParams)
    % estimate the extrinsic parameters according to the chessboard points and mask

    warning('off','all');

    Params = struct();
    Params.opt = [];
    Params.extrinsicMat = eye(4);
    estimationErrors = [];

    K = cameraParams.IntrinsicMatrix';

    imageSize = [cameraParams.ImageSize(2) cameraParams.ImageSize(1)];

    initExtrinsicParams = estimateExtrinsicParams.initExtrinsicParams;
    searchSpace = estimateExtrinsicParams.searchSpace;
    loss_type = estimateExtrinsicParams.loss_type;
    loss_threshold = estimateExtrinsicParams.loss_threshold;

    lower_bounds = initExtrinsicParams - searchSpace;
    upper_bounds = initExtrinsicParams + searchSpace;

    % parse the Distortion parameter
    if length(cameraParams.RadialDistortion) == 2
        D = [cameraParams.RadialDistortion 0 cameraParams.TangentialDistortion];
    elseif length(cameraParams.RadialDistortion) == 3
        D = [cameraParams.RadialDistortion cameraParams.TangentialDistortion];
    end

    global eval_count;
    eval_count = 0;

    global verbose;
    verbose = true;

    global opti_in_process;
    opti_in_process = {};

    global reprojectionDTError;

    % apply GA algorithms to solve the problem
    if isequal(loss_type, 'euler')
        min_objective = @(x) planeMaskMatchingEnergyLoss(x, chessboardLidarPoints, chessboardPlaneMaskDT, D, K, imageSize);
    elseif isequal(loss_type, 'se3')
        min_objective = @(x) planeMaskMatchingEnergyLossMex_(x, chessboardLidarPoints, chessboardPlaneMaskDT, D, K, imageSize);
    end

    options = gaoptimset('FitnessLimit', loss_threshold);

    [xopt, fmin, retcode, output,population,scores] = ga(min_objective, 6, [], [], [], [], lower_bounds, upper_bounds, [], options);

    extrinsicParams = xopt;

    if isequal(loss_type, 'euler')
        rmat = eul2rotm([extrinsicParams(1) extrinsicParams(2) extrinsicParams(3)]);
        vmat = [extrinsicParams(4) extrinsicParams(5) extrinsicParams(6)]';
        Tr_cam_2_lidar = eye(4);
        Tr_cam_2_lidar(1:3, 1:3) = rmat;
        Tr_cam_2_lidar(1:3, 4) = vmat;
    elseif isequal(loss_type, 'se3')
        extrinsicParams = eulerInv(se32euler(xopt));
        Tr_cam_2_lidar = euler2matrix(extrinsicParams);
    end
    
    Params.opt = extrinsicParams;
    Params.extrinsicMat = Tr_cam_2_lidar;

    varargout{1} = fmin;
    varargout{2} = retcode;
    varargout{3} = eval_count;
    varargout{4} = opti_in_process;

    estimationErrors = reprojectionDTError;

    function val = planeMaskMatchingEnergyLossMex_(x, chessboardLidarPoints, chessboardPlaneMaskDT, D, K, imageSize)
        global eval_count;
        eval_count = eval_count + 1;
        val = planeMaskMatchingEnergyLossMex(x', chessboardLidarPoints, chessboardPlaneMaskDT, D, K, imageSize);
        cam2lidar_euler = eulerInv(se32euler(x'));
        global verbose;
        if verbose
            fprintf('ga_optimize eval #%d: %.6f, [%.4f %.4f %.4f %.4f %.4f %.4f]\n', eval_count, ...
                                    val, ...
                                    cam2lidar_euler(1), ...
                                    cam2lidar_euler(2), ...
                                    cam2lidar_euler(3), ...
                                    cam2lidar_euler(4), ...
                                    cam2lidar_euler(5), ...
                                    cam2lidar_euler(6));
        end
        

    function val = planeMaskMatchingEnergyLoss(x, chessboardLidarPoints, chessboardPlaneMaskDT, D, K, imageSize)
        global eval_count;
        eval_count = eval_count + 1;

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
        global verbose;
        if verbose
            fprintf('ga_optimize eval #%d: %.6f, [%.4f %.4f %.4f %.4f %.4f %.4f]\n', eval_count, ...
                                    val, ...
                                    x(1), ...
                                    x(2), ...
                                    x(3), ...
                                    x(4), ...
                                    x(5), ...
                                    x(6));
        end

        global opti_in_process;
        opti_in_process{eval_count, 1} = eval_count;
        opti_in_process{eval_count, 2} = val;
        opti_in_process{eval_count, 3} = [x(1) x(2) x(3) x(4) x(5) x(6)];

        global reprojectionDTError;
        reprojectionDTError = val;
