function [Params, estimationErrors] = estimateExtrinsicParameters(  chessboardPoints, ...
                                                                    chessboardNormals, ...
                                                                    lidarPoints, ...
                                                                    lidarNormals, ...
                                                                    cameraLidarextrinsicCalibrationResult)
    % estimate the extrinsic parameters according to the chessboard points and normals

    addpath('/usr/local/MATLAB/R2017b/mex');
    addpath('.\thirdParty\nlopt\windows\mex');
    
    Params = struct();
    Params.opt = [];
    Params.extrinsicMat = eye(4);
    estimationErrors = [];

    if isempty(chessboardPoints) || isempty(chessboardNormals)
        warningIntrinsicCalibrationEmptyDialog();
        return;
    end

    % get the initial rotation via SVD 
    NN = chessboardNormals'*chessboardNormals;
    if isequal(det(NN), 0)
        warningNotEnoughPatternsDialog();
        return;
    end

    NM = chessboardNormals'*lidarNormals;
    UNR = NM;
    [U, S, V] = svd(UNR);
    InitRotation = V*U';

    % estimate the translation vector with constant rotation matrix
    opt.algorithm = NLOPT_LN_COBYLA;
    opt.verbose = 1;
    opt.lower_bounds = [-20, -20, -20];
    opt.upper_bounds = [20, 20, 20];
    opt.min_objective = @(x) calibrationCameraLidarChessboardTranslationalObjectiveFunc(x, InitRotation, chessboardNormals, chessboardPoints, lidarPoints, lidarNormals);
    opt.xtol_rel = 1e-4;
    opt.maxeval = 10000;

    %if ~isempty(cameraLidarextrinsicCalibrationResult.extrinsicMat)
    %    initTranslation = [cameraLidarextrinsicCalibrationResult.extrinsicMat(1, 4) ...
    %                    cameraLidarextrinsicCalibrationResult.extrinsicMat(2, 4) ...
    %                    cameraLidarextrinsicCalibrationResult.extrinsicMat(3, 4)];
    %else
    %    initTranslation = [0 0 0];
    %end
    initTranslation = [0 0 0];

    [xopt, fmin, retcode] = nlopt_optimize(opt, initTranslation);

    Params.extrinsicMat(1, 4) = xopt(1);
    Params.extrinsicMat(2, 4) = xopt(2);
    Params.extrinsicMat(3, 4) = xopt(3);

    % estimate the rotation matrix and translation matrix using the initial value
    eulerInit = double(rotm2eul(InitRotation));
    translationInit = xopt;
    opt.algorithm = NLOPT_LN_COBYLA;
    opt.verbose = 1;
    opt.lower_bounds = [-pi, -pi, -pi, -20, -20, -20];
    opt.upper_bounds = [pi, pi, pi, 20, 20, 20];
    opt.min_objective = @(x) calibrationCameraLidarChessboardObjectiveFunc(x, chessboardNormals, chessboardPoints, lidarPoints, lidarNormals);
    opt.xtol_rel = 1e-5;
    opt.maxeval = 10000;
    initTranformation = [eulerInit(1) eulerInit(2) eulerInit(3) translationInit(1) translationInit(2) translationInit(3)];
    [xopt, fmin, retcode] = nlopt_optimize(opt, initTranformation);

    % xopt = [-3.05 0.03 -1.68 -0.8 -0.5 -0.3];
    rotationMatrix = eul2rotm([xopt(1) xopt(2) xopt(3)]);
    Params.extrinsicMat(1:3, 1:3) = rotationMatrix;
    Params.extrinsicMat(1, 4) = xopt(4);
    Params.extrinsicMat(2, 4) = xopt(5);
    Params.extrinsicMat(3, 4) = xopt(6);
    estimationErrors = fmin;
    Params.opt = xopt;

    function [val, gradient] = calibrationCameraLidarChessboardTranslationalObjectiveFunc(x, InitRotation, chessboardNormals, chessboardPoints, lidarPoints, lidarNormals)
        chessboardNum = size(chessboardPoints, 1);
        vmat = [x(1) x(2) x(3)];
        normals = chessboardNormals * InitRotation';
        points = chessboardPoints * InitRotation' + ones(chessboardNum, 1) * vmat;
        count = 0;
        error_ = 0;
        for i = 1 : chessboardNum
            lidarPoints_ = lidarPoints{i};
            pointsNum = size(lidarPoints_, 1);
            count = count + pointsNum;

            duppoints = ones(pointsNum, 1) * points(i, :);
            dists = (lidarPoints_ - duppoints) * normals(i, :)';
            tmperror = dists' * dists;
            error_ = error_ + tmperror;
        end

        val = error_/count;
        if (nargout > 1)
            gradient = [0, 0, 0];
        end

    end

    function [val, gradient] = calibrationCameraLidarChessboardObjectiveFunc(x, chessboardNormals, chessboardPoints, lidarPoints, lidarNormals)
        % get the rotation matrix from the euler angles
        rmat = eul2rotm([x(1) x(2) x(3)]);
        vmat = [x(4) x(5) x(6)];
        
        chessboardNum = size(chessboardPoints, 1);

        normals = chessboardNormals * rmat';
        points = chessboardPoints * rmat' + ones(chessboardNum, 1) * vmat;
        count = 0;
        error_ = 0;
        for i = 1 : chessboardNum
            lidarPoints_ = lidarPoints{i};
            pointsNum = size(lidarPoints_, 1);
            count = count + pointsNum;

            duppoints = ones(pointsNum, 1) * points(i, :);
            dists = (lidarPoints_ - duppoints) * normals(i, :)';
            tmperror = dists' * dists;
            error_ = error_ + tmperror;
        end

        val = error_/count;
        if (nargout > 1)
            gradient = [0, 0, 0];
        end

    end

    function warningNotEnoughPatternsDialog
        str = sprintf('Not Enough Patterns');
        uiwait(warndlg(str, 'Not Enough Patterns'));
    end % warningNotEnoughPatternsDialog

    function warningIntrinsicCalibrationEmptyDialog
        str = sprintf('No Intrinsic Calibration Found');
        uiwait(warndlg(str, 'No Intrinsic Calibration Found'));
    end % warningIntrinsicCalibrationEmptyDialog
end