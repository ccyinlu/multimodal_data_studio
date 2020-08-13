classdef chessboard_camera_calibration < handle
    % camera_calibration() opens a toolstrip with docked figures to do the calibration of camera
    %
    % Example:
    %   Run "camera_calibration()"

    % Author(s): ethan
    % Copyright ccyinlu@whu.edu.cn
    % Date: 20190112

    properties (Transient = true)
        ToolGroup
        
        guiBrowserTable
        guiImageView
        guiImageDataView
        guiCalibrationResultsView

        current_dir
        current_images_dir
        current_undistorted_images_dir

        chessboardExtractionParams
        
        imageStruct
        cameraIntrinsicCalibrationData
        cameraIntrinsicCalibrationResult

        dataBrowserDownsampleRatio

        current_imageFilenames
        current_imageSize
        current_enables

        current_index
        current_image
        current_image_render_type
        current_image_show_type

        dataCursorImageHandle

        defaultBackGroundColor
    end

    methods

        function this = chessboard_camera_calibration(varargin)
            % create tool group
            import matlab.ui.internal.toolstrip.*  % for convenience below
            % set the path
            if isempty(varargin)
                this.current_dir = fileparts(which('chessboard_camera_calibration.m'));
            else
                this.current_dir = varargin{1};
            end

            addpath(fullfile(this.current_dir, './utils/interface'));
            addpath(fullfile(this.current_dir, './utils/display'));
            addpath(fullfile(this.current_dir, './figure'));
            addpath(fullfile(this.current_dir, './callback'));
            addpath(fullfile(this.current_dir, './calibration'));
            addpath(fullfile(this.current_dir, './thirdParty/yaml'));

            % mkdir the temp dir for thumnail images
            mkdir([this.current_dir '/.tmp']);

            %init the properties
            init(this);

            % create tool group
            this.ToolGroup = matlab.ui.internal.desktop.ToolGroup('chessboard camera calibration');
            addlistener(this.ToolGroup, 'GroupAction',@(src, event) closeCallback(this, event));

            % hidden the data browser
            this.ToolGroup.disableDataBrowser();

            % hidden the view tab
            this.ToolGroup.hideViewTab();

            % create figure
            this.guiBrowserTable    = createCameraCalibrationBrowserFigure(this);
            this.guiImageView       = createCameraCalibrationImageFigure(this);
            this.guiImageDataView   = createCameraCalibrationImageDataFigure(this);
            this.guiCalibrationResultsView = createCameraCalibrationResultsFigure(this);

            emptyFigure1            = createEmptyFigure(this);
            emptyFigure2            = createEmptyFigure(this);
            
            % create tab group 
            tabgroup = chessboard_camera_calibration_buildTabGroup(this);
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
            this.ToolGroup.addFigure(this.guiImageDataView.handle);
            this.ToolGroup.addFigure(this.guiCalibrationResultsView.handle);

            this.ToolGroup.addFigure(emptyFigure1.handle);
            this.ToolGroup.addFigure(emptyFigure2.handle);
            
            this.guiBrowserTable.handle.Visible             = 'on';
            this.guiImageView.handle.Visible                = 'on';
            emptyFigure1.handle.Visible                     = 'on';
            emptyFigure2.handle.Visible                     = 'on';
            this.guiImageDataView.handle.Visible            = 'on';
            
            this.guiCalibrationResultsView.handle.Visible   = 'on';

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
            if ~isempty(this.guiImageDataView.handle) && isvalid(this.guiImageDataView.handle)
                delete(this.guiImageDataView);
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
            MD.setDocumentColumnSpan(this.ToolGroup.Name, 0, 1, 2);
        end % setDefaultLayout

        function init(this)
            this.guiBrowserTable = [];
            this.guiImageView = [];
            this.guiImageDataView = [];

            % init the info
            initInfo(this);

            % suppress all the warnings
            warning('off');
        end % init

        function initParams(this)
            this.current_images_dir = this.current_dir;
            this.current_undistorted_images_dir = this.current_dir;

            this.dataBrowserDownsampleRatio = 0.05;

            this.current_image_render_type = 'range';
            this.current_image_show_type = 'raw';

            this.chessboardExtractionParams = struct();
            this.chessboardExtractionParams.chessboardProperties = [10 15 0.077];

            this.defaultBackGroundColor = [0.25 0.25 0.25];
        end % initParams

        function initData(this)
            this.imageStruct = [];

            this.current_imageFilenames = [];
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
            this.cameraIntrinsicCalibrationData.enable = [];

            this.cameraIntrinsicCalibrationResult = struct();
            this.cameraIntrinsicCalibrationResult.cameraParams = [];
            this.cameraIntrinsicCalibrationResult.imagesUsed = [];
            this.cameraIntrinsicCalibrationResult.estimationErrors = [];

            this.current_index = 1;
            this.current_image = [];

            this.dataCursorImageHandle = [];
        end % initData

        function initInfo(this)
            initParams(this);

            initData(this);
        end % initInfo

        function initFigure(this)

            set(this.guiImageView.handle, 'Color', this.defaultBackGroundColor);

            % set the data browser figure to gcf
            figure(this.guiBrowserTable.handle);
        end % initFigure

        function initView(this)
            % empty the data browser view
            set(this.guiBrowserTable.mTableImage, 'Data', {});
            % clear the figure of the image view
            cla(this.guiImageView.axesImageView);
            % empty the image point table
            set(this.guiImageDataView.mTableImagePoints, 'Data', {});
            % empty the calibrationResult table
            set(this.guiCalibrationResultsView.mTableCalibrationResults, 'Data', {});
        end % initView

        function updateImage(this)
            if ~isempty(this.current_imageFilenames) 
               if this.current_index <= length(this.current_imageFilenames)
                  % update the current_image
                  this.current_image = imread(this.current_imageFilenames{this.current_index});
               end
            end

        end % updateImage

        function updateImageView(this)
            % show the this.current_image to the this.guiImageView.handle
            if ~isempty(this.current_image)
                if isequal(this.current_image_show_type, 'raw')
                    imshow(this.current_image, 'Parent', this.guiImageView.axesImageView);
                elseif isequal(this.current_image_show_type, 'extracted')
                    if  ~isempty(this.cameraIntrinsicCalibrationData.imagePoints)
                        imshow(this.current_image, 'Parent', this.guiImageView.axesImageView);
                        hold(this.guiImageView.axesImageView, 'on');
                        chessboard_points = this.cameraIntrinsicCalibrationData.imagePoints{this.current_index};
                        plot(this.guiImageView.axesImageView, chessboard_points(:, 1), chessboard_points(:, 2), 'b+');
                        legend(this.guiImageView.axesImageView, 'corners detected in images');
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
                end
            else
                cla(this.guiImageView.axesImageView);
            end
            function warningNoneExtractionDataDialog
                str = sprintf('None Extraction Data Found! \r\nDo extraction before viewing the data');
                uiwait(warndlg(str, 'None Extraction Data'));
            end % warningNoneExtractionDataDialog

            function warningNoneIntrinsicCalibrationDialog
                str = sprintf('None Intrinsic Calinration Data Found! \r\nDo intrinsic calibration before viewing the undistorted data');
                uiwait(warndlg(str, 'None Intrinsic Calibration'));
            end % warningNoneIntrinsicCalibrationDialog
        end % updateImageView

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
            if ~isempty(this.cameraIntrinsicCalibrationData.imagePoints)
                imagePoits = this.cameraIntrinsicCalibrationData.imagePoints{this.current_index};
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
        end % generateCalibrationResultsTableData

        function refreshCalibrationResultsTableData(this)
            tableData = generateCalibrationResultsTableData(this);
            set(this.guiCalibrationResultsView.mTableCalibrationResults, 'Data', tableData);
        end % refreshCalibrationResultsTableData

    end
end



