function [lidarPoints, pointsNormal, boardSize, lidarUsed] = detectLidarChessboardPoints(lidarFilenames, algo, params)
    % detect the chessboard points according to the algos
    lidarPoints = {};
    pointsNormal = {};
    boardSize = {};
    lidarUsed = [];
    
    LidarNum = length(lidarFilenames);
    %global ifCancel;
    ifCancel = false;
    h = waitbar(0, 'detect the chessboard points', 'CreateCancelBtn',@waitbar_cancel, 'Name', 'analysis points');

    lidarUsed = zeros(LidarNum, 1);

    for i = 1 : LidarNum
        if ~ifCancel
            switch(algo)
            case 'diffPlane'
                if i > 1
                    % compare the point with the static point and extract the plane based on the differential points
                    [lidarPoints_, pointsNormal_, boardSize_, lidarUsed_] = detectLidarChessboardPointsDiffPlane(lidarFilenames{1}, ...
                                                                                                                 lidarFilenames{i}, ...
                                                                                                                 params);
                else
                    lidarUsed_ = false;
                end
            otherwise
                warningUnknownAlgoDialog();
                return
            end

            lidarUsed(i) = lidarUsed_;

            if lidarUsed_
                % cat the result
                lidarPoints = cat(1, lidarPoints, lidarPoints_);
                pointsNormal = cat(1, pointsNormal, pointsNormal_);
                boardSize = cat(1, boardSize, boardSize_);
            else
                lidarPoints = cat(1, lidarPoints, 0);
                pointsNormal = cat(1, pointsNormal, 0);
                boardSize = cat(1, boardSize, 0);
            end

            progress = i/LidarNum;
            msgs = sprintf('detecting chessboard points in points %d/%d', i, LidarNum);
            waitbar(progress, h, msgs, 'Name', 'analysis points');
        end
    end
    delete(h);
    lidarUsed = lidarUsed == 1;
    % show the results
    totalPointsFile = LidarNum;
    totalCalibPointsFile = sum(lidarUsed);
    rejectCalibPointsFile = totalPointsFile - totalCalibPointsFile;
    resultMsgsBox();

    function waitbar_cancel(src, event)
        %global ifCancel;
        ifCancel = true;
        delete(src);
    end

    function warningUnknownAlgoDialog
        str = sprintf('Specified algos not supported! \r\nPlease check your configuration');
        warndlg(str, 'Invalid Algo');
    end

    function resultMsgsBox
        str1 = sprintf('total points file:                                   %d', totalPointsFile);
        str2 = sprintf('points file for extrinsic calib:               %d', totalCalibPointsFile);
        str3 = sprintf('reject points file for extrinsic calib:     %d', rejectCalibPointsFile);
        uiwait(msgbox({str1 str2 str3}, 'Results'));
    end
end