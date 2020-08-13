classdef chessboard_camera_lidar_calibration < handle
    % camera_lidar_calibration() opens a toolstrip with docked figures to do the calibration between camera and lidar
    %
    % Example:
    %   Run "camera_lidar_calibration()"

    % Author(s): ethan
    % Copyright ccyinlu@whu.edu.cn
    % Date: 20190112

    properties (Transient = true)
        ToolGroup
        
        guiBrowserTable
        guiImageView
        guiPointView
        guiImageDataView
        guiPointDataView
        guiCalibrationResultsView

        current_dir
        current_images_dir
        current_points_dir
        current_intrinsic_dir
        current_extrinsic_dir

        chessboardLidarPointsExtractionAlgo
        ExtrinsicEstimationAlgo

        imagePointPairsStruct
        cameraIntrinsicCalibrationData
        cameraIntrinsicCalibrationResult
        cameraLidarextrinsicCalibrationData
        cameraLidarextrinsicCalibrationResult
        cameraLidarextrinsicCalibrationParams

        dataBrowserDownsampleRatio

        current_imageFilenames
        current_pointFilenames
        current_imageSize
        current_enables

        current_index
        current_image
        current_point
        current_point_intensity
        current_point_view
        current_point_lim
        current_point_Ilim
        current_point_size
        current_point_zoom_flag
        current_point_render_type
        current_image_render_type
        current_image_show_type
        current_reprojection_type

        current_camera2lidar_6dof

        dataCursorImageHandle
        dataCursorPointHandle

        defaultBackGroundColor

        extractChessboardPointsParams
        optimizationParams
    end

    methods

        function this = chessboard_camera_lidar_calibration(varargin)
            % create tool group
            import matlab.ui.internal.toolstrip.*  % for convenience below
            % set the path
            if isempty(varargin)
                this.current_dir = fileparts(which('chessboard_camera_lidar_calibration.m'));
            else
                this.current_dir = varargin{1};
            end
            addpath(fullfile(this.current_dir, './utils/interface'));
            addpath(fullfile(this.current_dir, './utils/display'));
            addpath(fullfile(this.current_dir, './utils/cloudSegmentation/func'));
            addpath(fullfile(this.current_dir, './utils/cloudSegmentation/mex'));
            addpath(fullfile(this.current_dir, './utils/optimization/func'));
            addpath(fullfile(this.current_dir, './utils/optimization/mex'));
            addpath(fullfile(this.current_dir, './figure'));
            addpath(fullfile(this.current_dir, './calibration'));
            addpath(fullfile(this.current_dir, './thirdParty/yaml'));

            % result = this.login();
            % if ~result
            %     return;
            % end

            % mkdir the temp dir for thumnail images
            mkdir([this.current_dir '/.tmp']);

            %init the properties
            init(this);

            % create tool group
            this.ToolGroup = matlab.ui.internal.desktop.ToolGroup('chessboard camera lidar calibration');
            addlistener(this.ToolGroup, 'GroupAction',@(src, event) closeCallback(this, event));

            % hidden the data browser
            this.ToolGroup.disableDataBrowser();

            % hidden the view tab
            this.ToolGroup.hideViewTab();

            % create figure
            this.guiBrowserTable    = createCameraLidarCalibrationBrowserFigure(this);
            this.guiImageView       = createCameraLidarCalibrationImageFigure(this);
            this.guiPointView       = createCameraLidarCalibrationPointFigure(this);
            this.guiImageDataView   = createCameraLidarCalibrationImageDataFigure(this);
            this.guiPointDataView   = createCameraLidarCalibrationPointDataFigure(this);
            this.guiCalibrationResultsView = createCameraLidarCalibrationResultsFigure(this);

            emptyFigure1            = createEmptyFigure(this);
            
            % create tab group 
            tabgroup = chessboard_camera_lidar_calibration_buildTabGroup(this);
            % add tab group to toolstrip (via tool group api)
            this.ToolGroup.addTabGroup(tabgroup);
            % select current tab (via tool group api)
            this.ToolGroup.SelectedTab = 'tabSequence';
            % render app
            this.ToolGroup.setPosition(100, 100, 1600, 820);
            this.ToolGroup.open;

            % add plot as a document
            this.ToolGroup.addFigure(this.guiBrowserTable.handle);
            this.ToolGroup.addFigure(this.guiImageView.handle);
            this.ToolGroup.addFigure(this.guiPointView.handle);
            this.ToolGroup.addFigure(this.guiImageDataView.handle);
            this.ToolGroup.addFigure(this.guiPointDataView.handle);
            this.ToolGroup.addFigure(this.guiCalibrationResultsView.handle);

            this.ToolGroup.addFigure(emptyFigure1.handle);
            
            this.guiBrowserTable.handle.Visible             = 'on';
            this.guiImageView.handle.Visible                = 'on';
            this.guiPointView.handle.Visible                = 'on';
            emptyFigure1.handle.Visible                     = 'on';
            this.guiImageDataView.handle.Visible            = 'on';
            
            this.guiCalibrationResultsView.handle.Visible   = 'on';
            this.guiPointDataView.handle.Visible            = 'on';

            setDefaultLayout(this);

            initFigure(this);

            initView(this);

            % store java toolgroup so that app will stay in memory
            internal.setJavaCustomData(this.ToolGroup.Peer, this);
        end

        function delete(this)
            if ~isempty(this.ToolGroup) && isvalid(this.ToolGroup)
                delete(this.ToolGroup);
            end

            if ~isempty(this.guiBrowserTable.handle) && isvalid(this.guiBrowserTable.handle)
                delete(this.guiBrowserTable);
            end
            if ~isempty(this.guiImageView.handle) && isvalid(this.guiImageView.handle)
                delete(this.guiImageView);
            end
            if ~isempty(this.guiImageView.handle) && isvalid(this.guiImageView.handle)
                delete(this.guiImageView);
            end
            if ~isempty(this.guiPointView.handle) && isvalid(this.guiPointView.handle)
                delete(this.guiPointView);
            end
            if ~isempty(this.guiImageDataView.handle) && isvalid(this.guiImageDataView.handle)
                delete(this.guiImageDataView);
            end
            if ~isempty(this.guiPointDataView.handle) && isvalid(this.guiPointDataView.handle)
                delete(this.guiPointDataView);
            end
            if ~isempty(this.guiCalibrationResultsView.handle) && isvalid(this.guiCalibrationResultsView.handle)
                delete(this.guiCalibrationResultsView);
            end
        end

        function closeCallback(this, event)
            ET = event.EventData.EventType;
            if strcmp(ET, 'CLOSED')
                delete(this);
            end
        end % closeCallback

        function setDefaultLayout(this)
            % default document layout
            MD = com.mathworks.mlservices.MatlabDesktopServices.getDesktop;
            pause(0.01);
            MD.setDocumentArrangement(this.ToolGroup.Name, 2, java.awt.Dimension(3,2));
            MD.setDocumentColumnWidths(this.ToolGroup.Name, [0.2 0.4 0.4]);
            MD.setDocumentRowHeights(this.ToolGroup.Name, [0.6 0.4]);
            MD.setDocumentRowSpan(this.ToolGroup.Name, 0, 0, 2);
        end % setDefaultLayout

        function init(this)
            this.guiBrowserTable = [];
            this.guiImageView = [];
            this.guiPointView = [];
            this.guiImageDataView = [];
            this.guiPointDataView = [];

            % init the info
            initInfo(this);

            % suppress all the warnings
            warning('off');
        end % init

        function initParams(this)
            this.current_images_dir = this.current_dir;
            this.current_points_dir = this.current_dir;
            this.current_intrinsic_dir = this.current_dir;
            this.current_extrinsic_dir = this.current_dir;
            
            this.ExtrinsicEstimationAlgo = 'Co-Mask-GA-LM';
            this.chessboardLidarPointsExtractionAlgo = 'diffPlane';

            this.dataBrowserDownsampleRatio = 0.05;

            this.current_point_view = struct();
            this.current_point_view.az = -145;
            this.current_point_view.el = 52;

            % specify the current [xLimMin xLimMax yLimMin yLimMax zLimMin zLimMax]
            this.current_point_lim = [-100 100 -100 100 -20 20];
            this.current_point_Ilim = [0 255];
            this.current_point_size = 5;
            this.current_point_render_type = 'plain';
            this.current_image_render_type = 'range';
            this.current_image_show_type = 'raw';
            this.current_reprojection_type = 'distort';

            this.defaultBackGroundColor = [0.25 0.25 0.25];

            this.extractChessboardPointsParams = struct();
            this.extractChessboardPointsParams.chessboardProperties = [7 9 0.1085];
            this.extractChessboardPointsParams.pointFileType = 'pcd';
            this.extractChessboardPointsParams.pointcloud_limit = [ -50 50 -100 0 -3 3];
            this.extractChessboardPointsParams.planeFitParams = {0.02 [0 1 0] 70}; 
            this.extractChessboardPointsParams.planeFitParams_stage_1 = {0.08 [0 1 0] 70};
            this.extractChessboardPointsParams.planeFitParams_stage_2 = {0.02 [0 1 0] 70};
            this.extractChessboardPointsParams.distanceMat_T = 0.05;
            this.extractChessboardPointsParams.ifRemoveGround = true;
            this.extractChessboardPointsParams.vertical_theta = [7.14299997649533 ...
                                                                6.14199999369268 ...
                                                                5.13599998302528 ...
                                                                4.12799999379798 ...
                                                                3.11499999759304 ...
                                                                2.10199999817724 ...
                                                                1.76700000074352 ...
                                                                1.42100000085773 ...
                                                                1.08699999931673 ...
                                                                0.751999999400975 ...
                                                                0.406000000141502 ...
                                                                0.0709999999638067 ...
                                                                -0.268000000278996 ...
                                                                -0.606000000378927 ...
                                                                -0.945000000723587 ...
                                                                -1.28399999783447 ...
                                                                -1.62100000004531 ...
                                                                -1.95999999838307 ...
                                                                -2.29799999910524 ...
                                                                -2.63600000254155 ...
                                                                -2.97400000194410 ...
                                                                -3.31100000267718 ...
                                                                -3.64799999962323 ...
                                                                -3.98599999721783 ...
                                                                -4.31900000176996 ...
                                                                -4.66099999339903 ...
                                                                -4.99499999970970 ...
                                                                -5.32600000224023 ...
                                                                -5.66700000201380 ...
                                                                -5.99999999161288 ...
                                                                -7.00099998042811 ...
                                                                -7.99700000286255 ...
                                                                -8.98699999592648 ...
                                                                -9.97100001148963 ...
                                                                -10.9479999800399 ...
                                                                -11.9169999834595 ...
                                                                -12.8739999942060 ...
                                                                -13.8250000366970 ...
                                                                -14.7660000283462 ...
                                                                -15.6960000066476];

            this.extractChessboardPointsParams.N_SCAN = 40;
            this.extractChessboardPointsParams.Horizon_SCAN = 1800;
            this.extractChessboardPointsParams.groundScanInd = 12;
            this.extractChessboardPointsParams.sensorMountAngle = 0;
            this.extractChessboardPointsParams.groundRemovalAngleT = 2;
            this.extractChessboardPointsParams.segmentTheta = 20/180.0*pi;
            this.extractChessboardPointsParams.feasibleSegmentValidPointNum = 30;
            this.extractChessboardPointsParams.segmentValidPointNum = 5;
            this.extractChessboardPointsParams.segmentValidLineNum = 3;
            this.extractChessboardPointsParams.horizontal_res = 0.2*pi/180;

            this.optimizationParams = struct();
            this.optimizationParams.ga_loss_threshold = 500;

            init_params_from_config_file(this);

        end % initParams

        function init_params_from_config_file(this)
            params_config_file = './camera_lidar_calibration_config.yml';
            if ~exist(params_config_file,'file')
                return;
            end

            params = YAML.read(params_config_file);
            this.extractChessboardPointsParams.chessboardProperties = str2num(params.chessboardProperties);
            this.extractChessboardPointsParams.pointFileType = params.pointFileType;
            this.extractChessboardPointsParams.pointcloud_limit = str2num(params.pointcloud_limit);
            planeFitParams_ = str2num(params.planeFitParams);
            this.extractChessboardPointsParams.planeFitParams = {planeFitParams_(1) planeFitParams_(2:4) planeFitParams_(5)}; 
            planeFitParams_stage_1_ = str2num(params.planeFitParams_stage_1);
            this.extractChessboardPointsParams.planeFitParams_stage_1 = {planeFitParams_stage_1_(1) planeFitParams_stage_1_(2:4) planeFitParams_stage_1_(5)}; 
            planeFitParams_stage_2_ = str2num(params.planeFitParams_stage_2);
            this.extractChessboardPointsParams.planeFitParams_stage_2 = {planeFitParams_stage_2_(1) planeFitParams_stage_2_(2:4) planeFitParams_stage_2_(5)}; 
            this.extractChessboardPointsParams.distanceMat_T = params.distanceMat_T;
            this.extractChessboardPointsParams.ifRemoveGround = params.ifRemoveGround;
            this.extractChessboardPointsParams.vertical_theta = str2num(params.vertical_theta);

            this.extractChessboardPointsParams.N_SCAN = params.N_SCAN;
            this.extractChessboardPointsParams.Horizon_SCAN = params.Horizon_SCAN;
            this.extractChessboardPointsParams.groundScanInd = params.groundScanInd;
            this.extractChessboardPointsParams.sensorMountAngle = params.sensorMountAngle;
            this.extractChessboardPointsParams.groundRemovalAngleT = params.groundRemovalAngleT;
            this.extractChessboardPointsParams.segmentTheta = params.segmentTheta;
            this.extractChessboardPointsParams.feasibleSegmentValidPointNum = params.feasibleSegmentValidPointNum;
            this.extractChessboardPointsParams.segmentValidPointNum = params.segmentValidPointNum;
            this.extractChessboardPointsParams.segmentValidLineNum = params.segmentValidLineNum;
            this.extractChessboardPointsParams.horizontal_res = params.horizontal_res;

            this.optimizationParams.ga_loss_threshold = params.ga_loss_threshold;
        end

        function initData(this)
            this.imagePointPairsStruct = [];

            this.current_imageFilenames = [];
            this.current_pointFilenames = [];
            this.current_imageSize = [];
            this.current_enables = [];

            this.cameraIntrinsicCalibrationData = struct();
            this.cameraIntrinsicCalibrationData.imagePoints = [];
            this.cameraIntrinsicCalibrationData.worldPoints = [];
            this.cameraIntrinsicCalibrationData.EstimateSkew = false;
            this.cameraIntrinsicCalibrationData.EsitmateTangentialDistortion = false;
            this.cameraIntrinsicCalibrationData.NumRadialDistortionCoefficients = 2;
            this.cameraIntrinsicCalibrationData.WorldUnits = 'meters';
            this.cameraIntrinsicCalibrationData.InitialIntrinsicMatrix = [];
            this.cameraIntrinsicCalibrationData.InitialRadialDistortion = [];
            this.cameraIntrinsicCalibrationData.ImageSize = [];

            this.cameraIntrinsicCalibrationResult = struct();
            this.cameraIntrinsicCalibrationResult.cameraParams = [];
            this.cameraIntrinsicCalibrationResult.imagesUsed = [];
            this.cameraIntrinsicCalibrationResult.estimationErrors = [];

            this.cameraLidarextrinsicCalibrationData = struct();
            this.cameraLidarextrinsicCalibrationData.imagePoints = [];
            this.cameraLidarextrinsicCalibrationData.chessboardPoints = [];
            this.cameraLidarextrinsicCalibrationData.chessboardNormals = [];
            this.cameraLidarextrinsicCalibrationData.chessboardSize = [];
            this.cameraLidarextrinsicCalibrationData.imageFilenames = [];
            this.cameraLidarextrinsicCalibrationData.lidarPoints = [];
            this.cameraLidarextrinsicCalibrationData.lidarPointsNormals = [];
            this.cameraLidarextrinsicCalibrationData.lidarFilenames = [];
            this.cameraLidarextrinsicCalibrationData.enable = [];
            % add the property of the plane mask 
            this.cameraLidarextrinsicCalibrationData.chessboardMask = [];

            this.cameraLidarextrinsicCalibrationResult = struct();
            this.cameraLidarextrinsicCalibrationResult.extrinsicMat = [];
            this.cameraLidarextrinsicCalibrationResult.estimationErrors = [];
            this.cameraLidarextrinsicCalibrationResult.opti_in_process = [];

            this.cameraLidarextrinsicCalibrationParams = struct();
            this.cameraLidarextrinsicCalibrationParams.initExtrinsicParams = [];
            this.cameraLidarextrinsicCalibrationParams.searchSpace = [];

            this.current_index = 1;
            this.current_image = [];
            this.current_point = [];
            this.current_point_intensity = [];
            this.current_camera2lidar_6dof = [0 0 0 0 0 0];

            this.dataCursorImageHandle = [];
            this.dataCursorPointHandle = [];
        end % initData

        function initInfo(this)
            initParams(this);

            initData(this);
        end % initInfo

        function initFigure(this)
            % axis the point view equal
            axis(this.guiPointView.axesPointView, 'equal');
            set(this.guiPointView.handle, 'Color', this.defaultBackGroundColor);
            set(this.guiPointView.axesPointView, 'Color', this.defaultBackGroundColor);
            set(this.guiPointView.axesPointView, 'Box', 'off');
            set(this.guiPointView.axesPointView, 'XColor', this.defaultBackGroundColor);
            set(this.guiPointView.axesPointView, 'YColor', this.defaultBackGroundColor);
            set(this.guiPointView.axesPointView, 'ZColor', this.defaultBackGroundColor);

            set(this.guiImageView.handle, 'Color', this.defaultBackGroundColor);

            % set the data browser figure to gcf
            figure(this.guiBrowserTable.handle);
        end % initFigure

        function initView(this)
            % empty the data browser view
            set(this.guiBrowserTable.mTableImage, 'Data', {});
            % clear the figure of the image view
            cla(this.guiImageView.axesImageView);
            % clear the figure of the point view
            cla(this.guiPointView.axesPointView); 
            % empty the image point table
            set(this.guiImageDataView.mTableImagePoints, 'Data', {});
            % empty the lidar point table
            set(this.guiPointDataView.mTableLidarPoints, 'Data', {});
            % empty the calibrationResult table
            set(this.guiCalibrationResultsView.mTableCalibrationResults, 'Data', {});
        end % initView

        function updateImagePoint(this)
            if ~isempty(this.current_imageFilenames) && ...
               ~isempty(this.current_pointFilenames)
                if this.current_index <= length(this.current_imageFilenames) && ...
                    this.current_index <= length(this.current_pointFilenames)
                    % update the current_image
                    this.current_image = imread(this.current_imageFilenames{this.current_index});
                    % update the current_point
                    switch (this.extractChessboardPointsParams.pointFileType)
                    case 'pcd'
                        tmp_pcd = pcread(this.current_pointFilenames{this.current_index});
                        this.current_point = tmp_pcd.Location;
                        this.current_point_intensity = tmp_pcd.Intensity;
                        tmp_pcd = [];
                    case 'bin'
                        fid = fopen(this.current_pointFilenames{this.current_index});
                        tmp_points = fread(fid, [4 inf], 'single')';
                        fclose(fid);
                        this.current_point = tmp_points(:, 1:3);
                        this.current_point_intensity = tmp_points(:, 4);
                        tmp_points = [];
                    otherwise
                        warningUnknownPointTypeDialog();
                        return
                    end
                end
            end

            function warningUnknownPointTypeDialog
                str = sprintf('Specified point types not supported! \r\nPlease check your configuration');
                warndlg(str, 'Invalid Point Type');
            end % warningUnknownPointTypeDialog
        end % updateImagePoint

        function updateImageView(this)
            % show the this.current_image to the this.guiImageView.handle
            if ~isempty(this.current_image)
                if isequal(this.current_image_show_type, 'raw')
                    imshow(this.current_image, 'Parent', this.guiImageView.axesImageView);
                elseif isequal(this.current_image_show_type, 'extracted')
                    if  ~isempty(this.cameraIntrinsicCalibrationData.imagePoints) && ~isempty(this.cameraLidarextrinsicCalibrationData.imagePoints)
                        imshow(this.current_image, 'Parent', this.guiImageView.axesImageView);
                        hold(this.guiImageView.axesImageView, 'on');
                        chessboard_points = this.cameraLidarextrinsicCalibrationData.imagePoints{this.current_index};
                        plot(this.guiImageView.axesImageView, chessboard_points(:, 1), chessboard_points(:, 2), 'b+');
                        legend(this.guiImageView.axesImageView, 'corners detected in images');
                    else
                        warningNoneExtractionDataDialog();
                    end
                elseif isequal(this.current_image_show_type, 'planeMasked')
                    if  ~isempty(this.cameraIntrinsicCalibrationData.imagePoints) && ~isempty(this.cameraLidarextrinsicCalibrationData.imagePoints)
                        imshow(this.current_image, 'Parent', this.guiImageView.axesImageView);
                        hold(this.guiImageView.axesImageView, 'on');

                        planeMaskedImage = this.current_image;
                        transparency = 0.5;
                        mask = this.cameraLidarextrinsicCalibrationData.chessboardMask{this.current_index};
                        mask3 = cat(3, mask, mask, mask);
                        mask_color = repmat([1 0 0], sum(mask(:)), 1); % red color
                        mask_color = uint8(mask_color(:) * transparency * 255); % transparency set to 0.5

                        planeMaskedImage(mask3) = uint8(min(this.current_image(mask3)*(1-transparency) + mask_color, 255));
                        imshow(planeMaskedImage, 'Parent', this.guiImageView.axesImageView);
                    else
                        warningNoneExtractionDataDialog();
                    end
                elseif isequal(this.current_image_show_type, 'undistorted')
                    if ~isempty(this.cameraIntrinsicCalibrationResult.cameraParams)
                        oringinalImage = this.current_image;
                        undistortedImage = undistortImage(oringinalImage, ...
                                                            this.cameraIntrinsicCalibrationResult.cameraParams);
                        imshow(undistortedImage, 'Parent', this.guiImageView.axesImageView);
                        legend(this.guiImageView.axesImageView, 'off');
                    else
                        warningNoneIntrinsicCalibrationDialog();
                    end
                elseif isequal(this.current_image_show_type, 'projectedPlane')
                    if ~isempty(this.cameraLidarextrinsicCalibrationData.lidarPoints)
                        if ~isempty(this.cameraIntrinsicCalibrationResult.cameraParams)
                            oringinalImage = this.current_image;
                            % CameraExtrinsicMat = this.cameraLidarextrinsicCalibrationResult.extrinsicMat;
                            % R_l2c = CameraExtrinsicMat(1:3, 1:3)';
                            % T_l2c = -R_l2c * CameraExtrinsicMat(1:3, 4);
                            R_l2c = (eul2rotm([this.current_camera2lidar_6dof(1) ...
                                                this.current_camera2lidar_6dof(2) ...
                                                this.current_camera2lidar_6dof(3)]))';
                            T_l2c = -R_l2c * [this.current_camera2lidar_6dof(4) ...
                                                this.current_camera2lidar_6dof(5) ...
                                                this.current_camera2lidar_6dof(6)]';

                            CameraMat = this.cameraIntrinsicCalibrationResult.cameraParams.IntrinsicMatrix;
                            K = CameraMat';
                            lidarPoints = this.cameraLidarextrinsicCalibrationData.lidarPoints{this.current_index};
                            undistortedImage = undistortImage(oringinalImage, ...
                                                                this.cameraIntrinsicCalibrationResult.cameraParams);

                            D = [this.cameraIntrinsicCalibrationResult.cameraParams.RadialDistortion this.cameraIntrinsicCalibrationResult.cameraParams.TangentialDistortion];

                            imshow(oringinalImage, 'Parent', this.guiImageView.axesImageView);
                            hold(this.guiImageView.axesImageView, 'on');
                            projectLidarPoints2Image(this.guiImageView.axesImageView, ...
                                                    oringinalImage, ...
                                                    lidarPoints', ...
                                                    [], ...
                                                    K, ...
                                                    R_l2c, ...
                                                    T_l2c, ...
                                                    D, ...
                                                    'plain', ...
                                                    this.current_reprojection_type, ...
                                                    this.extractChessboardPointsParams.pointcloud_limit, ...
                                                    [], ...
                                                    this.current_point_size);
                            legend(this.guiImageView.axesImageView, 'off');
                        else
                            warningNoneIntrinsicCalibrationDialog();
                        end
                    else
                        warningNoneExtractionLidarPointsDialog();
                    end
                elseif isequal(this.current_image_show_type, 'projectedAll')
                    if ~isempty(this.cameraIntrinsicCalibrationResult.cameraParams)
                        oringinalImage = this.current_image;
                        % CameraExtrinsicMat = this.cameraLidarextrinsicCalibrationResult.extrinsicMat;
                        % R_l2c = CameraExtrinsicMat(1:3, 1:3)';
                        % T_l2c = -R_l2c * CameraExtrinsicMat(1:3, 4);
                        R_l2c = (eul2rotm([this.current_camera2lidar_6dof(1) ...
                                                this.current_camera2lidar_6dof(2) ...
                                                this.current_camera2lidar_6dof(3)]))';
                        T_l2c = -R_l2c * [this.current_camera2lidar_6dof(4) ...
                                            this.current_camera2lidar_6dof(5) ...
                                            this.current_camera2lidar_6dof(6)]';

                        D = [this.cameraIntrinsicCalibrationResult.cameraParams.RadialDistortion this.cameraIntrinsicCalibrationResult.cameraParams.TangentialDistortion];
                        
                        CameraMat = this.cameraIntrinsicCalibrationResult.cameraParams.IntrinsicMatrix;
                        K = CameraMat';
                        % undistortedImage = undistortImage(oringinalImage, ...
                        %                                     this.cameraIntrinsicCalibrationResult.cameraParams);
                        imshow(oringinalImage, 'Parent', this.guiImageView.axesImageView);
                        hold(this.guiImageView.axesImageView, 'on');
                        projectLidarPoints2Image(this.guiImageView.axesImageView, ...
                                                oringinalImage, ...
                                                (this.current_point)', ...
                                                this.current_point_intensity, ...
                                                K, ...
                                                R_l2c, ...
                                                T_l2c, ...
                                                D, ...
                                                this.current_image_render_type, ...
                                                this.current_reprojection_type, ...
                                                this.extractChessboardPointsParams.pointcloud_limit, ...
                                                [0 50], ...
                                                this.current_point_size);
                        legend(this.guiImageView.axesImageView, 'off');
                    else
                        warningNoneIntrinsicCalibrationDialog();
                    end
                end
            else
                cla(this.guiImageView.axesImageView);
            end
            function warningNoneExtractionDataDialog
                str = sprintf('None Extraction Data Found! \r\nDo extraction before viewing the data');
                uiwait(warndlg(str, 'None Extraction Data'));
            end % warningNoneExtractionDataDialog

            function warningNoneIntrinsicCalibrationDialog
                str = sprintf('None Intrinsic Calinration Data Found!');
                uiwait(warndlg(str, 'None Intrinsic Calibration'));
            end % warningNoneIntrinsicCalibrationDialog

            function warningNoneExtrinsicCalibrationDialog
                str = sprintf('None Extrinsic Calibration Data Found! \r\nPlease check your configuration');
                uiwait(warndlg(str, 'None Extrinsic Calibration'));
            end % warningNoneExtrinsicCalibrationDialog

            function warningNoneExtractionLidarPointsDialog
                str = sprintf('None Extraction Lidar Points Found! \r\nPlease first extract the lidar points');
                uiwait(warndlg(str, 'None Extraction Lidar Points'));
            end % warningNoneExtractionLidarPointsDialog

        end % updateImageView

        function updatePointView(this)
            % show the this.current_point to the this.guiPointView.handle
            if ~isempty(this.current_point) && ~isempty(this.current_point_intensity)
                cla(this.guiPointView.axesPointView); 
                pointsShow(this.guiPointView.axesPointView, ...
                        [this.current_point this.current_point_intensity], ...
                        this.extractChessboardPointsParams.pointcloud_limit, ...
                        this.current_point_Ilim, ...
                        this.current_point_render_type, ...
                        this.current_point_size);
                % show the extracted points
                if ~isequal(this.current_image_show_type, 'raw')
                    if ~isempty(this.cameraLidarextrinsicCalibrationData.lidarPoints)
                        hold(this.guiPointView.axesPointView, 'on');
                        lidarPoints = this.cameraLidarextrinsicCalibrationData.lidarPoints{this.current_index};
                        scatter3(this.guiPointView.axesPointView, ...
                                lidarPoints(:, 1), ...
                                lidarPoints(:, 2), ...
                                lidarPoints(:, 3), ...
                                this.current_point_size * 1.5, ...
                                'r', 'filled');
                    end
                end
                view(this.guiPointView.axesPointView, this.current_point_view.az, this.current_point_view.el);
                drawnow;
            else
                cla(this.guiPointView.axesPointView); 
            end
        end % updatePointView

        function tableData = generateBrowserTableData(this)
            imageNum = length(this.current_imageFilenames);
            if exist([this.current_dir '/.tmp'], 'dir')
                % remove all in the current tmp
                delete([this.current_dir '/.tmp/*']);
                % create the thumnails of the images
                for i = 1 : imageNum
                    imageRawFilename = this.current_imageFilenames{i};
                    imageRaw = imread(imageRawFilename);
                    [~, id, ext] = fileparts(imageRawFilename);
                    imageResize = imresize(imageRaw, this.dataBrowserDownsampleRatio);
                    imageResizeFilename = sprintf('%s/.tmp/%s%s', this.current_dir, id, ext);
                    imwrite(imageResize, imageResizeFilename);
                end
            else
                warningTmpDirNotExistDialog();
            end

            this.guiBrowserTable.mTableImage.Units = 'pixels';
            currentWidth = this.guiBrowserTable.mTableImage.Position(3) * this.guiBrowserTable.mTableImageColumnWidthRatio(1);
            this.guiBrowserTable.mTableImage.Units = 'Normalized';
            currentHeight = currentWidth/this.current_imageSize(2) * this.current_imageSize(1);
            
            currentWidth = floor(currentWidth);
            currentHeight = floor(currentHeight);

            tableData = cell(imageNum, 3);
            for i = 1 : imageNum
                imageFilename = this.current_imageFilenames{i};
                [~, id, ext] = fileparts(imageFilename);
                imageThumnailFilename = sprintf('%s/.tmp/%s%s', this.current_dir, id, ext);
                imageFilenameShort = sprintf('<html><p>%d/%d<br/>%s%s</p></html>', i, imageNum, id, ext);
                % imageFilenameShort = [num2str(i) '/' num2str(imageNum) newline id ext];
                htmlUri = ['<html><img width=' ...
                            num2str(currentWidth) ...
                            ' height=' ...
                            num2str(currentHeight) ...
                            ' src="file:' ...
                            imageThumnailFilename ...
                            '"></html>'];
                tableData{i, 1} = htmlUri;
                tableData{i, 2} = imageFilenameShort;
                tableData{i, 3} = this.current_enables{i};
            end

            function warningTmpDirNotExistDialog
                str = sprintf('tempotory directory not exist! \r\nPlease check your configuration');
                h = warndlg(str, 'Tmp Not Found');
                uiwait(h);
            end
        end % generateBrowserTableData

        function refreshBrowserTableData(this)
            tableData = generateBrowserTableData(this);
            set(this.guiBrowserTable.mTableImage, 'Data', tableData);

            redrawBrowserTableHeight(this);
        end % refreshBrowserTableData

        function redrawBrowserTableHeight(this)
            if ~isempty(this.guiBrowserTable) && ~isempty(this.current_imageSize)
                this.guiBrowserTable.mTableImage.Units = 'pixels';
                currentWidth = this.guiBrowserTable.mTableImage.Position(3) * this.guiBrowserTable.mTableImageColumnWidthRatio(1);
                this.guiBrowserTable.mTableImage.Units = 'Normalized';
                currentHeight = currentWidth/this.current_imageSize(2) * this.current_imageSize(1);
                
                currentWidth = floor(currentWidth);
                currentHeight = floor(currentHeight);

                jscroll = findjobj(this.guiBrowserTable.mTableImage);
                jTable = jscroll.getViewport.getView;
                jTable.setRowHeight(currentHeight);

                imageNum = length(this.current_imageFilenames);
                tableData = cell(imageNum, 3);
                for i = 1 : imageNum
                    imageFilename = this.current_imageFilenames{i};
                    [~, id, ext] = fileparts(imageFilename);
                    imageThumnailFilename = sprintf('%s/.tmp/%s%s', this.current_dir, id, ext);
                    imageFilenameShort = sprintf('<html><p>%d/%d<br/>%s%s</p></html>', i, imageNum, id, ext);
                    % imageFilenameShort = [num2str(i) '/' num2str(imageNum) newline id ext];
                    htmlUri = ['<html><img width=' ...
                                num2str(currentWidth) ...
                                ' height=' ...
                                num2str(currentHeight) ...
                                ' src="file:' ...
                                imageThumnailFilename ...
                                '"></html>'];
                    tableData{i, 1} = htmlUri;
                    tableData{i, 2} = imageFilenameShort;
                    tableData{i, 3} = this.current_enables{i};
                end
                set(this.guiBrowserTable.mTableImage, 'Data', tableData);
            end
        end % redrawBrowserTableHeight

        function tableData = generateImagePointsTableData(this)
            if ~isempty(this.cameraLidarextrinsicCalibrationData.imagePoints)
                imagePoits = this.cameraLidarextrinsicCalibrationData.imagePoints{this.current_index};
                imagePointsNum = size(imagePoits, 1);

                tableData = cell(imagePointsNum, 2);
                for i = 1 : imagePointsNum
                    tableData{i, 1} = imagePoits(i, 1);
                    tableData{i, 2} = imagePoits(i, 2);
                end
            else
                tableData = [];
            end
            
        end % generateBrowserTableData

        function refreshImagePointsTableData(this)
            tableData = generateImagePointsTableData(this);
            if ~isempty(tableData)
                set(this.guiImageDataView.mTableImagePoints, 'Data', tableData);
            end
        end % refreshImagePointsTableData

        function tableData = generateLidarPointsTableData(this)
            if ~isempty(this.cameraLidarextrinsicCalibrationData.lidarPoints)
                lidarPoits = this.cameraLidarextrinsicCalibrationData.lidarPoints{this.current_index};
                lidarPointsNum = size(lidarPoits, 1);

                tableData = cell(lidarPointsNum, 3);
                for i = 1 : lidarPointsNum
                    tableData{i, 1} = lidarPoits(i, 1);
                    tableData{i, 2} = lidarPoits(i, 2);
                    tableData{i, 3} = lidarPoits(i, 3);
                end
            else
                tableData = [];
            end
            
        end % generateLidarPointsTableData

        function refreshLidarPointsTableData(this)
            tableData = generateLidarPointsTableData(this);
            if ~isempty(tableData)
                set(this.guiPointDataView.mTableLidarPoints, 'Data', tableData);
            end
        end % refreshLidarPointsTableData

        function tableData = generateCalibrationResultsTableData(this)
            tableData = cell(25, 3);
            current_line = 1;
            if ~isempty(this.cameraIntrinsicCalibrationResult.cameraParams)
                tableData{current_line, 1} = 'IntrinsicMatrix';
                K = this.cameraIntrinsicCalibrationResult.cameraParams.IntrinsicMatrix';
                for i = 1 : 3
                    for j = 1 : 3
                        tableData{i + current_line, j} = K(i, j);
                    end
                end
                current_line = current_line + 4;
                tableData{current_line, 1} = 'FocalLength';
                tableData{current_line, 2} = this.cameraIntrinsicCalibrationResult.cameraParams.FocalLength(1);
                tableData{current_line, 3} = this.cameraIntrinsicCalibrationResult.cameraParams.FocalLength(2);
                current_line = current_line + 1;
                tableData{current_line, 1} = 'PrincipalPoint';
                tableData{current_line, 2} = this.cameraIntrinsicCalibrationResult.cameraParams.PrincipalPoint(1);
                tableData{current_line, 3} = this.cameraIntrinsicCalibrationResult.cameraParams.PrincipalPoint(2);
                current_line = current_line + 1;
                tableData{current_line, 1} = 'RadialDistortion';
                tableData{current_line, 2} = this.cameraIntrinsicCalibrationResult.cameraParams.RadialDistortion(1);
                tableData{current_line, 3} = this.cameraIntrinsicCalibrationResult.cameraParams.RadialDistortion(2);
                current_line = current_line + 1;
                tableData{current_line, 1} = 'ImageSize';
                tableData{current_line, 2} = this.cameraIntrinsicCalibrationResult.cameraParams.ImageSize(1);
                tableData{current_line, 3} = this.cameraIntrinsicCalibrationResult.cameraParams.ImageSize(2);
                current_line = current_line + 1;
                tableData{current_line, 1} = 'MeanReprojectionError';
                tableData{current_line, 2} = this.cameraIntrinsicCalibrationResult.cameraParams.MeanReprojectionError;
            end

            if ~isempty(this.cameraLidarextrinsicCalibrationResult.estimationErrors)
                current_line = current_line + 2;

                camera2LidarRotation = this.cameraLidarextrinsicCalibrationResult.extrinsicMat(1:3, 1:3);
                camera2LidarEuler = rotm2eul(camera2LidarRotation);
                camera2LidarTranslation = this.cameraLidarextrinsicCalibrationResult.extrinsicMat(1:3, 4);
                tableData{current_line, 1} = 'camera2LidarRotation';
                for i = 1 : 3
                    for j = 1 : 3
                        tableData{i + current_line, j} = camera2LidarRotation(i, j);
                    end
                end
                current_line = current_line + 4;
                tableData{current_line, 1} = 'camera2LidarEuler';
                current_line = current_line + 1;
                for j = 1 : 3
                    tableData{current_line, j} = camera2LidarEuler(j);
                end
                current_line = current_line + 1;
                tableData{current_line, 1} = 'camera2LidarTranslation';
                current_line = current_line + 1;
                for j = 1 : 3
                    tableData{current_line, j} = camera2LidarTranslation(j);
                end
                current_line = current_line + 1;
                tableData{current_line, 1} = 'estimationErrors';
                tableData{current_line, 2} = this.cameraLidarextrinsicCalibrationResult.estimationErrors;
            end
        end % generateCalibrationResultsTableData

        function refreshCalibrationResultsTableData(this)
            tableData = generateCalibrationResultsTableData(this);
            set(this.guiCalibrationResultsView.mTableCalibrationResults, 'Data', tableData);
        end % refreshLidarPointsTableData

        function result = login(this)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % validate yourself using the account
            result = false;
            authorization_logo_image_filename = fullfile(this.current_dir, './icons/authorization_logo.png');

            d = figure('NumberTitle', 'off', ...
                        'MenuBar', 'none', ...
                        'ToolBar', 'none', ...
                        'Position',[700 400 250 200], ...
                        'Name','Authorization', ...
                        'Resize', 'off');
            
            txtAuthorizationLogo = uicontrol('Parent', d, ...
                                         'style', 'pushbutton', ...
                                         'Position', [2 108 245 90], ...
                                         'Callback', 'web http://www.in-driving.com/');
            % draw the multi_rosbag button image
            image = imread(authorization_logo_image_filename, 'BackGroundColor', [0.94 0.94 0.94]);
            image = imresize(image, fliplr(txtAuthorizationLogo.Position(1,3:4)));
            txtAuthorizationLogo.CData = image;

            jh = findjobj(txtAuthorizationLogo);
            jh.setFlyOverAppearance(true); % set the button boarder invisible

            uicontrol('Parent',d, ...
                    'Style','text', ...
                    'Position',[0 70 120 20], ...
                    'HorizontalAlignment', 'center', ...
                    'String','Account user');
            editBoxUser = uicontrol('Parent', d, ...
                                    'style', 'edit', ...
                                    'Position', [125 70 125 20], ...
                                    'HorizontalAlignment', 'left', ...
                                    'String', '');

            uicontrol('Parent',d, ...
                    'Style','text', ...
                    'Position',[0 44 120 20], ...
                    'HorizontalAlignment', 'center', ...
                    'String','Account password');
            
            jPass = javax.swing.JPasswordField;
            J_passwdin = javacomponent(jPass, [125 44 125 20]);

            uicontrol('Parent', d, ...
                        'Style','pushbutton',...
                        'Position', [50 12 50 25], ...
                        'String', 'Ok', ...
                        'callback', {@onOk, editBoxUser, J_passwdin});

            uicontrol('Parent', d, ...
                        'Style','pushbutton',...
                        'Position', [150 12 50 25], ...
                        'String', 'Cancel', ...
                        'Callback', 'delete(gcf)');

            movegui(d, 'center');
            uiwait(d);

            function onOk(~, ~, editBoxUser, J_passwdin)
                [ret_id, ret_info] = loginCheck(editBoxUser, J_passwdin);
                if isempty(ret_id)
                    return;
                end
                switch ret_id
                case 0
                    result = true;
                    delete(gcf);
                    return;
                otherwise
                    result = false;
                    warningDialog(ret_info);
                end

                function warningDialog(info)
                    str = sprintf('%s', info);
                    uiwait(warndlg(str, 'Authorization Result'));
                end % warningDialog
            end % onOk

            function [ret_id, ret_info] = loginCheck(hUser, hPass)
                import matlab.net.*;
                import matlab.net.http.*;
                % ret_id :
                % 0: authorization passed
                % 1: wrong user and password
                user = get(hUser,'String');
                pswd = hPass.text;

                if isempty(user) || isempty(pswd)
                    warningDialog();
                    ret_id = [];
                    ret_info = [];
                    return;
                end

                % r = jsonencode(struct('user', user, 'passwd', pswd));
                body = matlab.net.http.MessageBody(jsonencode(struct('user', user, 'passwd', pswd)));
                contentTypeField = matlab.net.http.field.ContentTypeField('text/plain');
                type1 = matlab.net.http.MediaType('text/*');
                type2 = matlab.net.http.MediaType('application/json', 'q', '.5');
                acceptField = matlab.net.http.field.AcceptField([type1 type2]);

                header = [acceptField contentTypeField];
                method = matlab.net.http.RequestMethod.POST;
                request = matlab.net.http.RequestMessage(method, header, body);
                uri = URI('http://47.100.39.180:8888');
                % uri = URI('https://www.mathworks.com/support/contact_us');

                try
                    resp = send(request, uri);
                catch e
                    if isa(e,'matlab.net.http.HTTPException')
                        warningConnectionDialog();
                        ret_id = [];
                        ret_info = [];
                        return;
                    end
                end

                status = resp.StatusCode;

                % get the return ret_id and ret_info
                data = jsondecode(resp.Body.Data{1});
                ret_id = data.ret_id;
                ret_info = data.ret_info;

                function warningDialog()
                    str = sprintf('username and password can not be empty');
                    uiwait(warndlg(str, 'username or password empty'));
                end % warningDialog

                function warningConnectionDialog()
                    str = sprintf('check your network or authorization service online');
                    uiwait(warndlg(str, 'Connection refused'));
                end % warningConnectionDialog
            end
            % validate yourself using the account
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end

        function test_load(this)
            fprintf('just to test the data_load\n');
            tmp = rand(10000);
        end

    end
end



