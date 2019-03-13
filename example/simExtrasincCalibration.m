
addpath('../calibration');
N = 100;
xoptGroundTruth = [-3.05 0.03 -1.68 -0.8 -0.5 -0.3];
lidarNormalsNoise = 0.1;
chessboardPointsNoise = [-3 3 -1 1 7 10];
global lidarPointsNumMean;
lidarPointsNumMean = 500;
global lidarPointsNumStd;
lidarPointsNumStd = 10;
global lidarPointsXSpread;
lidarPointsXSpread = 0.5;
global lidarPointsZSpread;
lidarPointsZSpread = 0.5;

lidarPointsCenterNoise = 0.1;
global lidarPointsPlaneNoise;
lidarPointsPlaneNoise = 0.1;


rmatGroundTruth = eul2rotm([xoptGroundTruth(1) xoptGroundTruth(2) xoptGroundTruth(3)]);
vmatGroundTruth = [xoptGroundTruth(4) xoptGroundTruth(5) xoptGroundTruth(6)];
TmatGroundTruth = eye(4);
TmatGroundTruth(1:3, 1:3) = rmatGroundTruth;
TmatGroundTruth(1, 4) = vmatGroundTruth(1);
TmatGroundTruth(2, 4) = vmatGroundTruth(2);
TmatGroundTruth(3, 4) = vmatGroundTruth(3);

chessboardNormalsBase = [0 0 1];
eulerBase = [0 0 0];
euler = ones(N, 1) * eulerBase + random('unif', -pi/6, pi/6, [N, 3]);

chessboardNormals = zeros(N, 3);
lidarNormals = cell(N, 1);
for i = 1 : N
    rotm = eul2rotm(euler(i, :));
    chessboardNormals(i, :) = chessboardNormalsBase * rotm';
    lidarNormals{i} = chessboardNormals(i, :) * rmatGroundTruth';
    lidarNormals{i} = lidarNormals{i} + random('unif', -lidarNormalsNoise, lidarNormalsNoise, [1, 3]);
end

lidarPoints = cell(N, 1);
chessboardPoints = zeros(N, 3);

for i = 1 : N
    chessboardPoints(i, :) = [  random('unif', chessboardPointsNoise(1), chessboardPointsNoise(2), 1) ...
                                random('unif', chessboardPointsNoise(3), chessboardPointsNoise(4), 1) ...
                                random('unif', chessboardPointsNoise(5), chessboardPointsNoise(6), 1)];
    LidarPointCenter = chessboardPoints(i, :)*rmatGroundTruth' + vmatGroundTruth;
    LidarPointCenter = LidarPointCenter + [ random('unif', -lidarPointsCenterNoise, lidarPointsCenterNoise, 1) ...
                                                random('unif', -lidarPointsCenterNoise, lidarPointsCenterNoise, 1) ...
                                                random('unif', -lidarPointsCenterNoise, lidarPointsCenterNoise, 1)];
    lidarPoints{i} = generateLidarPoints(LidarPointCenter, lidarNormals{i});
end

cameraLidarextrinsicCalibrationData = struct();
cameraLidarextrinsicCalibrationData.chessboardPoints = chessboardPoints;
cameraLidarextrinsicCalibrationData.chessboardNormals = chessboardNormals;
cameraLidarextrinsicCalibrationData.lidarPointsNormals = lidarNormals;
cameraLidarextrinsicCalibrationData.lidarPoints = lidarPoints;

% ESTIMATE THE EXTRINSIC MATRIX
[Params, ~] = estimateExtrinsicParameters(cameraLidarextrinsicCalibrationData, []);

disp('errorOpt:');
errorOpt = Params.opt - xoptGroundTruth;
disp(errorOpt);

errorMatrix = Params.extrinsicMat - TmatGroundTruth;
disp('error matrix');
disp(errorMatrix);

function lidarPoint = generateLidarPoints(pointCenter, pointNormal)
    global lidarPointsNumMean;
    global lidarPointsNumStd;
    global lidarPointsXSpread;
    global lidarPointsZSpread;
    global lidarPointsPlaneNoise;

    pointsNum = floor(random('norm', lidarPointsNumMean, lidarPointsNumStd));
    lidarPoint = zeros(pointsNum, 3);
    for i = 1 : pointsNum
        x = random('unif', pointCenter(1) - lidarPointsXSpread, pointCenter(1) + lidarPointsXSpread, 1);
        z = random('unif', pointCenter(3) - lidarPointsZSpread, pointCenter(3) + lidarPointsZSpread, 1);
        y = (pointCenter*pointNormal' - pointNormal(1)*x - pointNormal(3)*z)/pointNormal(2);
        lidarPoint(i, :) = [x y z];
        
        lidarPoint(i, :) = lidarPoint(i, :) + [ random('unif', -lidarPointsPlaneNoise, lidarPointsPlaneNoise, 1) ...
                                                random('unif', -lidarPointsPlaneNoise, lidarPointsPlaneNoise, 1) ...
                                                random('unif', -lidarPointsPlaneNoise, lidarPointsPlaneNoise, 1)];
    end
    lidarPointDiff = lidarPoint(2:end, :) - lidarPoint(1:end-1, :);
    normalError = norm(lidarPointDiff*pointNormal');
    disp(normalError);
end