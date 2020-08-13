function [lidarPoints, pointsNormal, boardSize, lidarUsed] = detectLidarChessboardPointsDiffPlane(lidarBaseFilename, ...
                                                                                                  lidarObjectFilename, ...
                                                                                                  params)
    % extract the plane according to the differential points
    lidarPoints = [];
    pointsNormal = [];
    boardSize = [];
    lidarUsed = [];

    pointFileType           = params.pointFileType;

    % first read the points and convert to points to pcd
    pcBase = [];
    pcObject = [];
    switch(pointFileType)
    case 'pcd'
        pcBase = pcread(lidarBaseFilename);
        pcObject = pcread(lidarObjectFilename);
    otherwise
        warningUnknownPointTypeDialog();
        return
    end

    [isFound, chessboardPoints, normal] = extractChessboardPoints(pcBase, ...
                                                                  pcObject, ...
                                                                  params);

    if isFound
        lidarPoints = chessboardPoints;
        pointsNormal = normal;
        % extract the board size info
        boardSize = [];
    end

    lidarUsed = isFound;

    function warningUnknownPointTypeDialog
        str = sprintf('Specified point types not supported! \r\nPlease check your configuration');
        warndlg(str, 'Invalid Point Type');
    end % warningUnknownPointTypeDialog
end