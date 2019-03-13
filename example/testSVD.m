
N = 100;
eulerGroundTruth = [pi, 0, pi/2];
rotmGroundTruth = eul2rotm(eulerGroundTruth);

chessboardNormalsBase = [0 0 1];
eulerBase = [0 0 0];
euler = ones(N, 1) * eulerBase + random('unif', -pi/6, pi/6, [N, 3]);

chessboardNormals = zeros(N, 3);
lidarNormals = zeros(N, 3);
for i = 1 : N
    rotm = eul2rotm(euler(i, :));
    chessboardNormals(i, :) = chessboardNormalsBase * rotm';
    lidarNormals(i, :) = chessboardNormals(i, :) * rotmGroundTruth';
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
svdEuler = rotm2eul(InitRotation);