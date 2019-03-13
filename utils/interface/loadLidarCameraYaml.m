function [CameraExtrinsicMat, CameraMat, DistCoeff, ImageSize, ReprojectionError] = loadLidarCameraYaml(filePath)
    % Author: ethan
    % Email: ccyinlu@whu.edu.cn
    % Date: 2018-10-30

    %% load the config file which includes the intrinsic and extrinsic between the camera and lidar
    %% config format description:

    %% %YAML:1.0
    %% ---
    %% CameraExtrinsicMat: !!opencv-matrix
    %%   rows: 4
    %%   cols: 4
    %%   dt: d
    %%   data: [ 4x4 ]
    %% CameraMat: !!opencv-matrix
    %%   rows: 3
    %%   cols: 3
    %%   dt: d
    %%   data: [ 3x3 ]
    %% DistCoeff: !!opencv-matrix
    %%   rows: 1
    %%   cols: 5
    %%   dt: d
    %%   data: [ 1x5 ]
    %% ImageSize: [ W, H ]
    %% ReprojectionError: double

    % we use YAMLMATLAB tools to parse the YAML file
    % @installation: Install it by  addpath(genpath('path/to/codes'));
    % @attention: for the autoware generated yml file, you should manually delete the fist comment line and '!!opencv-matrix'
    % reference URL: https://shan2011.blogspot.com/2013/05/transferring-data-from-opencv-to-matlab.html
    % reference URL: http://vision.is.tohoku.ac.jp/~kyamagu/software/yaml/

    % the calibration matrix from the camera to lidar
    calib = YAML.read(filePath);
    CameraExtrinsicMat = reshape(calib.CameraExtrinsicMat.data, calib.CameraExtrinsicMat.rows, calib.CameraExtrinsicMat.cols);
    CameraExtrinsicMat = CameraExtrinsicMat'; % matlab matrix will form the matrix along the cols order

    % camera intrinsic matrix
    CameraMat = reshape(calib.CameraMat.data, calib.CameraMat.rows, calib.CameraMat.cols);
    CameraMat = CameraMat'; % matlab matrix will form the matrix along the cols order

    % Distortion Coefficient matrix
    DistCoeff = reshape(calib.DistCoeff.data, calib.DistCoeff.rows, calib.DistCoeff.cols);

    % image size
    ImageSize = calib.ImageSize;

    % reprojection error
    % for more information about the reprojection error
    % refer to the Autoware guide "Calibtaion Toolbox Manual.pdf"
    ReprojectionError = calib.ReprojectionError;
end
