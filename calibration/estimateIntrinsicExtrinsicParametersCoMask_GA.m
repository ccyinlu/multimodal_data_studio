function [Params, estimationErrors, varargout] = estimateIntrinsicExtrinsicParametersCoMask_GA(  chessboardLidarPoints, ...
                                                                                                chessboardPlaneMaskDT, ...
                                                                                                estimateIntrinsicExtrinsicParams)
% estimate the extrinsic parameters according to the chessboard points and mask

    warning('off','all');

    Params = struct();
    Params.opt = [];
    Params.extrinsicMat = eye(4);
    estimationErrors = [];

    initIntrinsicExtrinsicParams = estimateIntrinsicExtrinsicParams.initIntrinsicExtrinsicParams;
    searchSpace = estimateIntrinsicExtrinsicParams.searchSpace;
    image_size = size(chessboardPlaneMaskDT{1});
    imageSize = [image_size(2) image_size(1)];

    lower_bounds = initIntrinsicExtrinsicParams - searchSpace;
    upper_bounds = initIntrinsicExtrinsicParams + searchSpace;

    global eval_count;
    eval_count = 0;

    global verbose;
    verbose = true;

    % apply GA algorithms to solve the problem
    min_objective = @(x) planeMaskMatchingEnergyLoss(x, chessboardLidarPoints, chessboardPlaneMaskDT, imageSize);
    [xopt, fmin, retcode, output,population,scores] = ga(min_objective, length(initIntrinsicExtrinsicParams), [], [], [], [], lower_bounds, upper_bounds);

    extrinsicParams = xopt(1:6);

    rmat = eul2rotm([extrinsicParams(1) extrinsicParams(2) extrinsicParams(3)]);
    vmat = [extrinsicParams(4) extrinsicParams(5) extrinsicParams(6)]';
    Tr_cam_2_lidar = eye(4);
    Tr_cam_2_lidar(1:3, 1:3) = rmat;
    Tr_cam_2_lidar(1:3, 4) = vmat;
    
    Params.opt = xopt;
    Params.extrinsicMat = Tr_cam_2_lidar;
    Params.fx = xopt(7);
    Params.fy = xopt(8);
    Params.cx = xopt(9);
    Params.cy = xopt(10);
    Params.k1 = xopt(11);
    Params.k2 = xopt(12);

    varargout{1} = fmin;
    varargout{2} = retcode;
    varargout{3} = eval_count;

    estimationErrors = fmin;        

    function val = planeMaskMatchingEnergyLoss(x, chessboardLidarPoints, chessboardPlaneMaskDT, imageSize)
        global eval_count;
        eval_count = eval_count + 1;

        k1 = x(11);
        k2 = x(12);
        k3 = 0;
        p1 = 0;
        p2 = 0;

        % parse the K
        fx = x(7);
        fy = x(8);
        cx = x(9);
        cy = x(10);

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
            fprintf('ga_optimize eval #%d: %-12.6f, [%-7.4f %-7.4f %-7.4f %-7.4f %-7.4f %-7.4f %-9.4f %-9.4f %-8.4f %-8.4f %-6.4f %-6.4f ]\n', eval_count, ...
                                    val, ...
                                    x(1), ...
                                    x(2), ...
                                    x(3), ...
                                    x(4), ...
                                    x(5), ...
                                    x(6), ...
                                    x(7), ...
                                    x(8), ...
                                    x(9), ...
                                    x(10), ...
                                    x(11), ...
                                    x(12));
        end
