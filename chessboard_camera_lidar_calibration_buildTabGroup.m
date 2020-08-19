function tabgroup = chessboard_camera_lidar_calibration_buildTabGroup(app)
    % Build the TabGroup for the ros_data_multiBag_image_point.

    % Author(s): ethan
    % Copyright ccyinlu@whu.edu.cn.
    % Date: 20190102
    import matlab.ui.internal.toolstrip.*
    % tab group
    tabgroup = TabGroup();
    % tabs
    tabCalibration = Tab('Calibration');
    tabCalibration.Tag = 'tabCalibration';

    tabView = Tab('View');
    tabView.Tag = 'tabView';

    createFile(tabCalibration, app);
    createCalibrationView(tabCalibration, app);
    createIntrinsicOption(tabCalibration, app);
    createExtrinsicOption(tabCalibration, app);
    createCalibration(tabCalibration, app)

    createViewControl(tabView, app)

    % assemble
    tabgroup.add(tabCalibration);
    tabgroup.add(tabView);
end

function createFile(tab, app)
    import matlab.ui.internal.toolstrip.*
    % create section
    section = Section('FILE');
    section.Tag = 'FileSection';
    % create column
    column1 = Column();
    column2 = Column();
    column3 = Column();
    column4 = Column();
    % add file add push button
    % built-in: see matlab.ui.internal.toolstrip.Icon.showStandardIcons()
    PairsAddButton = SplitButton(sprintf('Add\nPairs'), Icon(javaObjectEDT('javax.swing.ImageIcon', fullfile(app.current_dir, 'icons', 'add_chessboard_30.png'))));
    PairsAddButton.Tag = 'PairsAddButton';
    PairsAddButton.Description = 'add image-point pairs';
    PairsAddButton.DynamicPopupFcn = @(x,y) buildPairsAddButtonDynamicPopupList_SmallIcon(app, 'swing');
    % add file close push button
    % built-in: see Icon.showStandardIcons()
    PairClearButton = Button(sprintf('Clear\nPairs'), Icon(javaObjectEDT('javax.swing.ImageIcon', fullfile(app.current_dir, 'icons', 'clear_chessboard_30.png'))));
    PairClearButton.Tag = 'PairClearButton';
    PairClearButton.Description = 'clear the pair';

    % add load calibration file button
    % built-in: see Icon.showStandardIcons()
    LoadCalibrationButton = Button(sprintf('Load\nExtrinsic'), Icon.ADD_24);
    LoadCalibrationButton.Tag = 'LoadExtrinsic';
    LoadCalibrationButton.Description = 'Load Extrinsic yml';

    % add load intrinsic file button
    % built-in: see Icon.showStandardIcons()
    LoadIntrinsicButton = Button(sprintf('Load\nIntrinsic'), Icon.ADD_24);
    LoadIntrinsicButton.Tag = 'LoadIntrinsic';
    LoadIntrinsicButton.Description = 'Load Intrinsic yml';

    % assemble
    add(tab, section);
    add(section, column1);
    add(column1, PairsAddButton);
    add(section, column2);
    add(column2, PairClearButton);
    add(section, column3);
    add(column3, LoadCalibrationButton);
    add(section, column4);
    add(column4, LoadIntrinsicButton);

    % add callback
    PairClearButton.ButtonPushedFcn = @(x, y) clearPair(x, y, app);
    LoadCalibrationButton.ButtonPushedFcn = @(x, y) onLoadCalibration(x, y, app);
    LoadIntrinsicButton.ButtonPushedFcn = @(x, y) onLoadIntrinsic(x, y, app);

    %-------------------------------------------------------------------------%
    function clearPair(src, data, app)
        % init the image render type
        app.current_image_show_type = 'raw';

        % init data
        app.initData();

        % empty the figure and table
        app.initView();
    end  % clearPair

    %-------------------------------------------------------------------------%
    function onLoadCalibration(src, data, app)
        % choose a folder to save the configuration file
        [FileName, PathName] = uigetfile('*.yml;*.yaml;*.txt', 'load calibration parameters file', [app.current_extrinsic_dir]);
        if isequal(FileName, 0)
            disp(myLog('user selected cancel'));
            return;
        else
            disp(myLog(['user selected ', [PathName FileName]]));
        end
        app.current_extrinsic_dir = PathName;
        [CameraExtrinsicMat, CameraMat, DistCoeff, ImageSize, ReprojectionError] = loadLidarCameraYaml([PathName FileName]);
        % format the calibration result
        if isequal(length(DistCoeff), 5)
            RadialDistortion = [DistCoeff(1) DistCoeff(2) DistCoeff(3)];
            TangentialDistortion = [DistCoeff(4) DistCoeff(5)];
        elseif isequal(length(DistCoeff), 4)
            RadialDistortion = [DistCoeff(1) DistCoeff(2)];
            TangentialDistortion = [DistCoeff(3) DistCoeff(4)];
        elseif isequal(length(DistCoeff), 3)
            RadialDistortion = [DistCoeff(1) DistCoeff(2) DistCoeff(3)];
            TangentialDistortion = [0 0];
        elseif isequal(length(DistCoeff), 2)
            RadialDistortion = [DistCoeff(1) DistCoeff(2)];
            TangentialDistortion = [0 0];
        end
        
        cameraParams = cameraParameters('IntrinsicMatrix', CameraMat', ...
                                        'RadialDistortion', RadialDistortion, ...
                                        'TangentialDistortion', TangentialDistortion, ...
                                        'ImageSize', [ImageSize(1) ImageSize(2)]);
        app.cameraIntrinsicCalibrationResult.cameraParams = cameraParams;
        app.cameraLidarextrinsicCalibrationResult.extrinsicMat = CameraExtrinsicMat;
        app.cameraLidarextrinsicCalibrationResult.estimationErrors = ReprojectionError;

        % update the current_camera2lidar_6dof
        CameraExtrinsicMat = app.cameraLidarextrinsicCalibrationResult.extrinsicMat;
        R_c2l = rotm2eul(CameraExtrinsicMat(1:3, 1:3));
        T_c2l = CameraExtrinsicMat(1:3, 4);
        app.current_camera2lidar_6dof(1) = R_c2l(1);
        app.current_camera2lidar_6dof(2) = R_c2l(2);
        app.current_camera2lidar_6dof(3) = R_c2l(3);
        app.current_camera2lidar_6dof(4) = T_c2l(1);
        app.current_camera2lidar_6dof(5) = T_c2l(2);
        app.current_camera2lidar_6dof(6) = T_c2l(3);

    end  % onLoadCalibration    

    %-------------------------------------------------------------------------%
    function onLoadIntrinsic(src, data, app)
        % choose a folder to save the configuration file
        [FileName, PathName] = uigetfile('*.yml', 'load intrinsic parameters file', [app.current_intrinsic_dir]);
        if isequal(FileName, 0)
            disp(myLog('user selected cancel'));
            return;
        else
            disp(myLog(['user selected ', [PathName FileName]]));
        end
        app.current_intrinsic_dir = PathName;
        [CameraMat, DistCoeff, ImageSize] = loadCameraYaml([PathName FileName]);
        % format the calibration result
        if isequal(length(DistCoeff), 5)
            RadialDistortion = [DistCoeff(1) DistCoeff(2) DistCoeff(3)];
            TangentialDistortion = [DistCoeff(4) DistCoeff(5)];
        elseif isequal(length(DistCoeff), 4)
            RadialDistortion = [DistCoeff(1) DistCoeff(2)];
            TangentialDistortion = [DistCoeff(3) DistCoeff(4)];
        elseif isequal(length(DistCoeff), 3)
            RadialDistortion = [DistCoeff(1) DistCoeff(2) DistCoeff(3)];
            TangentialDistortion = [0 0];
        elseif isequal(length(DistCoeff), 2)
            RadialDistortion = [DistCoeff(1) DistCoeff(2)];
            TangentialDistortion = [0 0];
        end
        
        cameraParams = cameraParameters('IntrinsicMatrix', CameraMat', ...
                                        'RadialDistortion', RadialDistortion, ...
                                        'TangentialDistortion', TangentialDistortion, ...
                                        'ImageSize', [ImageSize(1) ImageSize(2)]);
        app.cameraIntrinsicCalibrationResult.cameraParams = cameraParams;

    end  % onLoadIntrinsic    
end

function popup = buildPairsAddButtonDynamicPopupList_SmallIcon(app, mode)
    import matlab.ui.internal.toolstrip.*
    popup = PopupList();
    item = ListItem('add from file', Icon(javaObjectEDT('javax.swing.ImageIcon', fullfile(app.current_dir, 'icons', 'add_chessboard_16.png'))));
    item.Description = 'add from file';
    item.ShowDescription = false;
    item.ItemPushedFcn = @(x, y) addPairsFromFile(x, y, app);
    popup.add(item);
    item = ListItem('add from rosbag', Icon(javaObjectEDT('javax.swing.ImageIcon', fullfile(app.current_dir, 'icons', 'add_ros_16.png'))));
    item.Description = 'add from rosbag';
    item.ShowDescription = false;
    item.ItemPushedFcn = @(x, y) addPairsFromRosbag(x, y, app);
    popup.add(item);

    %-------------------------------------------------------------------------%
    function addPairsFromFile(src, data, app)
        % create a dialog that get the images-points pairs
        d = dialog('Position',[700 400 500 200],'Name','Load Images-Points Pairs');

        chessboardSquareMetric = 'millimeters';
        chessboardSquareValue = app.extractChessboardPointsParams.chessboardProperties(3) * 1000;
        chessboardSquareNumX = app.extractChessboardPointsParams.chessboardProperties(1);
        chessboardSquareNumY = app.extractChessboardPointsParams.chessboardProperties(2);

        % add controls to load the images
        txtImagesFolder     = uicontrol('Parent',d, ...
                                        'Style','text', ...
                                        'Units', 'normalized', ...
                                        'Position',[0.0 0.85 - 0.05 0.6 0.15], ...
                                        'HorizontalAlignment', 'left', ...
                                        'String','Folder for images from the Camera:');

        editBoxImagesFolder = uicontrol('Parent', d, ...
                                        'style', 'edit', ...
                                        'Units', 'normalized', ...
                                        'Position', [0.0 0.7 0.8 0.15], ...
                                        'HorizontalAlignment', 'left', ...
                                        'String', app.current_images_dir);
        
        buttonBrowserImagesFolder = uicontrol('Parent', d, ...
                                            'style', 'pushbutton', ...
                                            'Units', 'normalized', ...
                                            'Position', [0.8 0.7 0.2 0.15], ...
                                            'String', 'browser', ...
                                            'callback', @onBrowserImageFolder);

        % add control to load the points
        txtPointsFolder     = uicontrol('Parent',d, ...
                                            'Style','text', ...
                                            'Units', 'normalized', ...
                                            'Position',[0.0 0.55 - 0.05 0.6 0.15], ...
                                            'HorizontalAlignment', 'left', ...
                                            'String','Folder for points from the Lidar:');
        
        editBoxPointsFolder = uicontrol('Parent', d, ...
                                        'style', 'edit', ...
                                        'Units', 'normalized', ...
                                        'Position', [0.0 0.4 0.8 0.15], ...
                                        'HorizontalAlignment', 'left', ...
                                        'String', app.current_points_dir);
        
        buttonBrowserPointsFolder = uicontrol('Parent', d, ...
                                            'style', 'pushbutton', ...
                                            'Units', 'normalized', ...
                                            'Position', [0.8 0.4 0.2 0.15], ...
                                            'String', 'browser', ...
                                            'callback', @onBrowserPointFolder);

        % add control to set the chessboard parameters
        % add control to load the points
        txtChessboardSquare      = uicontrol('Parent', d, ...
                                            'Style', 'text', ...
                                            'Units', 'normalized', ...
                                            'Position', [0.0 0.25 - 0.05 0.45 0.15], ...
                                            'HorizontalAlignment', 'left', ...
                                            'String', 'Size of checkerboard square: ');
        
        editBoxChessboardSquare  = uicontrol('Parent', d, ...
                                            'style', 'edit', ...
                                            'Units', 'normalized', ...
                                            'Position', [0.45 0.25 0.09 0.12], ...
                                            'HorizontalAlignment', 'center', ...
                                            'String', num2str(chessboardSquareValue), ...
                                            'callback', @onEditChessboardSquareValue);

        popupChessboardMetric    = uicontrol('Parent', d,...
                                            'Style','popupmenu',...
                                            'Units', 'normalized', ...
                                            'Position', [0.55 0.25 - 0.03 0.2 0.15], ...
                                            'String',{'millimeters';'centimeters'},...
                                            'Callback', @onSelectChessboardMetric);

        editBoxChessboardSquareNumX    = uicontrol('Parent', d,...
                                                    'style', 'edit', ...
                                                    'Units', 'normalized', ...
                                                    'Position', [0.78 0.25 0.09 0.12], ...
                                                    'HorizontalAlignment', 'center', ...
                                                    'String', num2str(chessboardSquareNumX), ...
                                                    'callback', @onEditChessboardSquareNumX);

        txtChessboardSquareNumSetting      = uicontrol('Parent', d, ...
                                                    'Style', 'text', ...
                                                    'Units', 'normalized', ...
                                                    'Position', [0.875 0.25 - 0.05 0.03 0.15], ...
                                                    'HorizontalAlignment', 'center', ...
                                                    'String', 'x');

        editBoxChessboardSquareNumY    = uicontrol('Parent', d,...
                                                    'style', 'edit', ...
                                                    'Units', 'normalized', ...
                                                    'Position', [0.91 0.25 0.09 0.12], ...
                                                    'HorizontalAlignment', 'center', ...
                                                    'String', num2str(chessboardSquareNumY), ...
                                                    'callback', @onEditChessboardSquareNumY);

        pushbuttonOk            = uicontrol('Parent', d,...
                                            'Style','pushbutton',...
                                            'Units', 'normalized', ...
                                            'Position', [0.2 0.05 0.15 0.15], ...
                                            'String', 'Ok', ...
                                            'callback', @onOk);

        pushbuttonCancel            = uicontrol('Parent', d,...
                                            'Style','pushbutton',...
                                            'Units', 'normalized', ...
                                            'Position', [0.6 0.05 0.15 0.15], ...
                                            'String', 'Cancel', ...
                                            'Callback', 'delete(gcf)');


        function onBrowserImageFolder(src, event)
            % get the dir to load the images
            folder_name = uigetdir(app.current_images_dir, 'Select folder to load the images');
            if isequal(folder_name, 0)
                disp(myLog('user selected cancel'));
                return;
            else
                disp(myLog(['user selected ', folder_name]));
                app.current_images_dir = folder_name;
                % update the edit box
                set(editBoxImagesFolder, 'String', app.current_images_dir);
            end
        end % onBrowserImageFolder

        function onBrowserPointFolder(src, event)
            % get the dir to load the images
            folder_name = uigetdir(app.current_points_dir, 'Select folder to load the points');
            if isequal(folder_name, 0)
                disp(myLog('user selected cancel'));
                return;
            else
                disp(myLog(['user selected ', folder_name]));
                app.current_points_dir = folder_name;
                % update the edit box
                set(editBoxPointsFolder, 'String', app.current_points_dir);
            end
        end % onBrowserPointFolder

        function onEditChessboardSquareValue(src, event)
            % change the start time
            value = str2num(src.String);
            if value >= 0 & value <= 1000
                chessboardSquareValue = value;
                set(editBoxChessboardSquare ,'String', sprintf('%d', chessboardSquareValue));
            end
        end % onEditChessboardSquareValue

        function onEditChessboardSquareNumX(src, event)
            % change the start time
            value = str2num(src.String);
            if value >= 1 & value <= 100
                chessboardSquareNumX = value;
                set(editBoxChessboardSquareNumX ,'String', sprintf('%d', chessboardSquareNumX));
            end
        end % onEditChessboardSquareNumX

        function onEditChessboardSquareNumY(src, event)
            % change the start time
            value = str2num(src.String);
            if value >= 1 & value <= 100
                chessboardSquareNumY = value;
                set(editBoxChessboardSquareNumY ,'String', sprintf('%d', chessboardSquareNumY));
            end
        end % onEditChessboardSquareNumY

        function onSelectChessboardMetric(src,event)
            idx = src.Value;
            popup_items = src.String;
            % This code uses dot notation to get properties.
            % Dot notation runs in R2014b and later.
            % For R2014a and earlier:
            % idx = get(popup,'Value');
            % popup_items = get(popup,'String');
            chessboardSquareMetric = char(popup_items(idx,:));
        end % onSelectChessboardMetric

        function onOk(src, event)
            % update the variables
            switch(chessboardSquareMetric)
            case 'millimeters'
                app.extractChessboardPointsParams.chessboardProperties(3) = chessboardSquareValue / 1000;
            case 'centimeters'
                app.extractChessboardPointsParams.chessboardProperties(3) = chessboardSquareValue / 100;
            otherwise % default the metric to millimeters
                app.extractChessboardPointsParams.chessboardProperties(3) = chessboardSquareValue / 1000;
            end

            % set the pattern size of the chessboard
            app.extractChessboardPointsParams.chessboardProperties(1) = chessboardSquareNumX;
            app.extractChessboardPointsParams.chessboardProperties(2) = chessboardSquareNumY;

            % delete the figure
            delete(gcf);

            % init the image render type
            app.current_image_show_type = 'raw';

            % init data
            app.initData();

            % empty the figure and table
            app.initView();

            % load the image-point pairs
            imagePointPairsStruct = loadImagePointPairs(app.current_images_dir, app.current_points_dir);
            if isempty(imagePointPairsStruct)
                return
            end

            app.imagePointPairsStruct = imagePointPairsStruct;

            app.current_imageFilenames = imagePointPairsStruct.rawImagesFilename;
            app.current_pointFilenames = imagePointPairsStruct.rawPointsFilename;
            app.current_enables = cell(length(app.current_imageFilenames));
            for i = 1 : length(app.current_enables)
                app.current_enables{i} = false;
            end

            % update the current_imageSize
            app.current_imageSize = size(imread(app.current_imageFilenames{1}));

            % refresh the content of the browser table
            app.refreshBrowserTableData();

            app.current_index = 1;

            app.updateImagePoint();
            % refresh the camera image chessboard points
            app.updateImageView();
            % refresh the lidar point chessboard points
            app.updatePointView();

            % refresh the image points table view
            app.refreshImagePointsTableData();

            % refresh the lidar points table view
            app.refreshLidarPointsTableData();
        end % onOK

    end  % addPairsFromFile

    %-------------------------------------------------------------------------%
    function addPairsFromRosbag(src, data, app)
        % create a dialog for loading the pair sequence

    end  % addPairsFromRosbag
end

function createCalibrationView(tab, app)
    import matlab.ui.internal.toolstrip.*
    % create section
    section = Section('View');
    section.Tag = 'ViewCalibrationSection';
    % create column
    % column1 = Column('Width', 100, 'HorizontalAlignment', 'center');
    column1 = Column('HorizontalAlignment', 'center');
    column2 = Column();
    column3 = Column();
    
    labelDownsampleRatio = Label('PreviewRatio:');
    editDownsampleRatio = EditField();
    editDownsampleRatio.Value = num2str(app.dataBrowserDownsampleRatio);
    editDownsampleRatio.Description = 'DownsampleRatio';
    editDownsampleRatio.ValueChangedFcn =  @onEditDownsampleRatioChangedCallback;

    group = matlab.ui.internal.toolstrip.ButtonGroup();
    buttonImageRaw = RadioButton(group, 'raw');
    buttonImageRaw.Description = 'raw';
    buttonImageRaw.ValueChangedFcn = @imageTypeRawPropertyChangedCallback;

    buttonImageExtracted = RadioButton(group, 'extracted');
    buttonImageExtracted.Description = 'extracted';
    buttonImageExtracted.ValueChangedFcn = @imageTypeExtractedPropertyChangedCallback;

    buttonImageUndistorted = RadioButton(group, 'undistorted');
    buttonImageUndistorted.Description = 'undistorted';
    buttonImageUndistorted.ValueChangedFcn = @imageTypeUndistortedPropertyChangedCallback;

    buttonImagePlaneMasked = RadioButton(group, 'planeMasked');
    buttonImagePlaneMasked.Description = 'planeMasked';
    buttonImagePlaneMasked.ValueChangedFcn = @imageTypePlaneMaskedPropertyChangedCallback;

    buttonImageProjectedPlane = RadioButton(group, 'projected plane');
    buttonImageProjectedPlane.Description = 'projectedPlane';
    buttonImageProjectedPlane.ValueChangedFcn = @imageTypeProjectedPlanePropertyChangedCallback;

    buttonImageProjectedAll = RadioButton(group, 'projected all');
    buttonImageProjectedAll.Description = 'projectedAll';
    buttonImageProjectedAll.ValueChangedFcn = @imageTypeProjectedAllPropertyChangedCallback;

    % assemble
    add(tab, section);
    add(section, column1);
    add(column1, labelDownsampleRatio);
    add(column1, editDownsampleRatio);

    add(section, column2);
    add(column2, buttonImageRaw);
    add(column2, buttonImageExtracted);
    add(column2, buttonImageUndistorted);

    add(section, column3);
    add(column3, buttonImagePlaneMasked);
    add(column3, buttonImageProjectedPlane);
    add(column3, buttonImageProjectedAll);

    buttonImageRaw.Value = true;

    function onEditDownsampleRatioChangedCallback(src, event)
        value = str2double(src.Text);
        if value > 0 & value < 1
            app.dataBrowserDownsampleRatio = value;
            app.refreshBrowserTableData();
        else
            src.Text = num2str(app.dataBrowserDownsampleRatio);
        end
    end % oneditDownsampleRatioChangedCallback

    function imageTypeRawPropertyChangedCallback(src, data)
        if data.EventData.NewValue
            app.current_image_show_type = 'raw';
            app.updateImageView();
            app.updatePointView();

            if isequal(app.current_image_show_type, 'raw')
                buttonImageRaw.Value = true;
            elseif isequal(app.current_image_show_type, 'extracted')
                buttonImageExtracted.Value = true;
            elseif isequal(app.current_image_show_type, 'undistorted')
                buttonImageUndistorted.Value = true;
            elseif isequal(app.current_image_show_type, 'projectedPlane')
                buttonImageProjectedPlane.Value = true;
            elseif isequal(app.current_image_show_type, 'projectedAll')
                buttonImageProjectedAll.Value = true;
            else
                buttonImageRaw.Value = true;
            end
        end
    end % imageTypeRawPropertyChangedCallback

    function imageTypeExtractedPropertyChangedCallback(src, data)
        if data.EventData.NewValue
            if ~isequal(app.current_image_show_type, 'extracted')
                if ~isempty(app.cameraIntrinsicCalibrationData.imagePoints) && ~isempty(app.cameraLidarextrinsicCalibrationData.imagePoints)
                    app.current_image_show_type = 'extracted';
                    app.updateImageView();
                    app.updatePointView();
                else
                    warningNoneExtractionDataDialog();
                end
            end

            if isequal(app.current_image_show_type, 'raw')
                buttonImageRaw.Value = true;
            elseif isequal(app.current_image_show_type, 'extracted')
                buttonImageExtracted.Value = true;
            elseif isequal(app.current_image_show_type, 'undistorted')
                buttonImageUndistorted.Value = true;
            elseif isequal(app.current_image_show_type, 'projectedPlane')
                buttonImageProjectedPlane.Value = true;
            elseif isequal(app.current_image_show_type, 'projectedAll')
                buttonImageProjectedAll.Value = true;
            else
                buttonImageRaw.Value = true;
            end
        end

        function warningNoneExtractionDataDialog
            str = sprintf('None Extraction Data Found! \r\nDo extraction before viewing the data');
            uiwait(warndlg(str, 'None Extraction Data'));
        end % warningNoneExtractionDataDialog
    end % imageTypeExtractedPropertyChangedCallback

    function imageTypeUndistortedPropertyChangedCallback(src, data)
        if data.EventData.NewValue
            if ~isequal(app.current_image_show_type, 'undistorted')
                if ~isempty(app.cameraIntrinsicCalibrationResult.cameraParams)
                    app.current_image_show_type = 'undistorted';
                    app.updateImageView();
                    app.updatePointView();
                else
                    warningNoneIntrinsicCalibrationDialog();
                end
            end
            
            if isequal(app.current_image_show_type, 'raw')
                buttonImageRaw.Value = true;
            elseif isequal(app.current_image_show_type, 'extracted')
                buttonImageExtracted.Value = true;
            elseif isequal(app.current_image_show_type, 'undistorted')
                buttonImageUndistorted.Value = true;
            elseif isequal(app.current_image_show_type, 'projectedPlane')
                buttonImageProjectedPlane.Value = true;
            elseif isequal(app.current_image_show_type, 'projectedAll')
                buttonImageProjectedAll.Value = true;
            else
                buttonImageRaw.Value = true;
            end
        end

        function warningNoneIntrinsicCalibrationDialog
            str = sprintf('None Intrinsic Calibration Data Found! \r\nDo intrinsic calibration before viewing the undistorted data');
            uiwait(warndlg(str, 'None Intrinsic Calibration'));
        end % warningNoneIntrinsicCalibrationDialog
    end % imageTypeUndistortedPropertyChangedCallback

    function imageTypePlaneMaskedPropertyChangedCallback(src, data)
        if data.EventData.NewValue
            if ~isequal(app.current_image_show_type, 'planeMasked')
                if ~isempty(app.cameraLidarextrinsicCalibrationData.chessboardMask)
                    app.current_image_show_type = 'planeMasked';
                    app.updateImageView();
                    app.updatePointView();
                else
                    warningNonePlaneMaskDialog();
                end
            end
            
            if isequal(app.current_image_show_type, 'raw')
                buttonImageRaw.Value = true;
            elseif isequal(app.current_image_show_type, 'extracted')
                buttonImageExtracted.Value = true;
            elseif isequal(app.current_image_show_type, 'undistorted')
                buttonImageUndistorted.Value = true;
            elseif isequal(app.current_image_show_type, 'planeMasked')
                buttonImagePlaneMasked.Value = true;
            elseif isequal(app.current_image_show_type, 'projectedPlane')
                buttonImageProjectedPlane.Value = true;
            elseif isequal(app.current_image_show_type, 'projectedAll')
                buttonImageProjectedAll.Value = true;
            else
                buttonImageRaw.Value = true;
            end
        end

        function warningNonePlaneMaskDialog
            str = sprintf('None Plane Mask Found');
            uiwait(warndlg(str, 'extract plane mask first'));
        end % warningNonePlaneMaskDialog
    end % imageTypePlaneMaskedPropertyChangedCallback

    function imageTypeProjectedPlanePropertyChangedCallback(src, data)
        if data.EventData.NewValue
            if ~isequal(app.current_image_show_type, 'projectedPlane')
                % if isempty(app.cameraLidarextrinsicCalibrationResult.estimationErrors)
                %     warningNoneExtrinsicCalibrationDialog();
                % elseif isempty(app.cameraLidarextrinsicCalibrationData.lidarPoints)
                %     warningNoneExtractionDataDialog();
                % else
                    app.current_image_show_type = 'projectedPlane';
                    app.updateImageView();
                    app.updatePointView();
                % end
            end

            if isequal(app.current_image_show_type, 'raw')
                buttonImageRaw.Value = true;
            elseif isequal(app.current_image_show_type, 'extracted')
                buttonImageExtracted.Value = true;
            elseif isequal(app.current_image_show_type, 'undistorted')
                buttonImageUndistorted.Value = true;
            elseif isequal(app.current_image_show_type, 'projectedPlane')
                buttonImageProjectedPlane.Value = true;
            elseif isequal(app.current_image_show_type, 'projectedAll')
                buttonImageProjectedAll.Value = true;
            else
                buttonImageRaw.Value = true;
            end
        end

        function warningNoneExtractionDataDialog
            str = sprintf('None Extraction Data Found! \r\nDo extraction before viewing the data');
            uiwait(warndlg(str, 'None Extraction Data'));
        end % warningNoneExtractionDataDialog

        function warningNoneExtrinsicCalibrationDialog
            str = sprintf('None Extrinsic Calinration Data Found! \r\nDo extrinsic calibration before viewing the projected data');
            uiwait(warndlg(str, 'None Extrinsic Calibration'));
        end % warningNoneExtrinsicCalibrationDialog
    end % imageTypeProjectedPlanePropertyChangedCallback

    function imageTypeProjectedAllPropertyChangedCallback(src, data)
        if data.EventData.NewValue
            if ~isequal(app.current_image_show_type, 'projectedAll')
                % if ~isempty(app.cameraIntrinsicCalibrationResult.cameraParams) && ~isempty(app.cameraLidarextrinsicCalibrationResult.estimationErrors)
                    app.current_image_show_type = 'projectedAll';
                    app.updateImageView();
                    app.updatePointView();
                % else
                %     warningNoneExtrinsicCalibrationDialog();
                % % end
            end

            if isequal(app.current_image_show_type, 'raw')
                buttonImageRaw.Value = true;
            elseif isequal(app.current_image_show_type, 'extracted')
                buttonImageExtracted.Value = true;
            elseif isequal(app.current_image_show_type, 'undistorted')
                buttonImageUndistorted.Value = true;
            elseif isequal(app.current_image_show_type, 'projectedPlane')
                buttonImageProjectedPlane.Value = true;
            elseif isequal(app.current_image_show_type, 'projectedAll')
                buttonImageProjectedAll.Value = true;
            else
                buttonImageRaw.Value = true;
            end
        end

        function warningNoneExtrinsicCalibrationDialog
            str = sprintf('None Extrinsic Calinration Data Found! \r\nDo extrinsic calibration before viewing the projected data');
            uiwait(warndlg(str, 'None Extrinsic Calibration'));
        end % warningNoneExtrinsicCalibrationDialog
    end % imageTypeProjectedAllPropertyChangedCallback
end

function createIntrinsicOption(tab, app)
    import matlab.ui.internal.toolstrip.*
    % create section
    section = Section('Intrinsic Option');
    section.Tag = 'IntrinsicOptionSection';
    % create column
    column1 = Column();
    column2 = Column();
    
    % add radio option for intrinsic calibration
    group = matlab.ui.internal.toolstrip.ButtonGroup();
    labelDistortionCoefficients = Label('Radial Distortion:');
    buttonDistortionCoefficients2 = RadioButton(group, '2 Coeffs');
    buttonDistortionCoefficients2.Description = 'Coefficients2';
    buttonDistortionCoefficients2.ValueChangedFcn = @DistortionCoefficientsPropertyChangedCallback;
    buttonDistortionCoefficients3 = RadioButton(group, '3 Coeffs');
    buttonDistortionCoefficients3.Description = 'Coefficients3';
    buttonDistortionCoefficients3.ValueChangedFcn = @DistortionCoefficientsPropertyChangedCallback;

    % add checkbox option for intrinsic calibration
    labelCompute = Label('Compute:');
    checkboxSkew = CheckBox('Skew');
    checkboxSkew.Description = 'if enable skew';
    checkboxSkew.ValueChangedFcn = @checkboxSkewPropertyChangedCallback;
    checkboxTangentialDistortion = CheckBox('Tan Dist');
    checkboxTangentialDistortion.Description = 'if Tangential Distortion';
    checkboxTangentialDistortion.ValueChangedFcn = @checkboxTangentialDistortionPropertyChangedCallback;

    % assemble
    add(tab, section);
    add(section, column1);
    add(column1, labelDistortionCoefficients);
    add(column1, buttonDistortionCoefficients2);
    add(column1, buttonDistortionCoefficients3);
    add(section, column2);
    add(column2, labelCompute);
    add(column2, checkboxSkew);
    add(column2, checkboxTangentialDistortion);

    if isequal(app.cameraIntrinsicCalibrationData.NumRadialDistortionCoefficients, 2)
        buttonDistortionCoefficients2.Value = true;
    else
        buttonDistortionCoefficients3.Value = true;
    end

    if app.cameraIntrinsicCalibrationData.EstimateSkew
        checkboxSkew.Value = true;
    else
        checkboxSkew.Value = false;
    end

    if app.cameraIntrinsicCalibrationData.EsitmateTangentialDistortion
        checkboxTangentialDistortion.Value = true;
    else
        checkboxTangentialDistortion.Value = false;
    end

    function DistortionCoefficientsPropertyChangedCallback(src, data)
        if isequal(src.Description, 'Coefficients2')
            app.cameraIntrinsicCalibrationData.NumRadialDistortionCoefficients = 2;
        elseif isequal(src.Description, 'Coefficients3')
            app.cameraIntrinsicCalibrationData.NumRadialDistortionCoefficients = 3;
        end
    end % DistortionCoefficientsPropertyChangedCallback

    function checkboxSkewPropertyChangedCallback(src, data)
        if isequal(data.EventData.NewValue, 1)
            app.cameraIntrinsicCalibrationData.EstimateSkew = true;
        else
            app.cameraIntrinsicCalibrationData.EstimateSkew = false;
        end
    end % checkboxSkewPropertyChangedCallback

    function checkboxTangentialDistortionPropertyChangedCallback(src, data)
        if isequal(data.EventData.NewValue, 1)
            app.cameraIntrinsicCalibrationData.EsitmateTangentialDistortion = true;
        else
            app.cameraIntrinsicCalibrationData.EsitmateTangentialDistortion = false;
        end
    end % checkboxTangentialDistortionPropertyChangedCallback

end % createIntrinsicOption

function createExtrinsicOption(tab, app)
    import matlab.ui.internal.toolstrip.*
    % create section
    section = Section('Extrinsic Option');
    section.Tag = 'ExtrinsicOptionSection';
    % create column
    % column1 = Column('Width', 80, 'HorizontalAlignment', 'center');
    column1 = Column('HorizontalAlignment', 'center');
    column2 = Column('Width', 100, 'HorizontalAlignment', 'center');
    % column3 = Column('Width', 100, 'HorizontalAlignment', 'right');
    column3 = Column('HorizontalAlignment', 'right');
    column4 = Column('Width', 60);
    % column5 = Column('Width', 100, 'HorizontalAlignment', 'right');
    column5 = Column('HorizontalAlignment', 'right');
    column6 = Column('Width', 60);
    % column7 = Column('Width', 100, 'HorizontalAlignment', 'right');
    column7 = Column('HorizontalAlignment', 'right');
    column8 = Column('Width', 60);
    
    % add listbox for point file type selection
    % add render type
    labelPointFileType = Label('pointFile:');
    labelPointChessboardAlogo = Label('pointAlgo:');
    labelExtrinsicExtractionAlogo = Label('ExtAlgo:');

    dropDownPointFileType = DropDown({'P' 'pcd'; 'B' 'bin'});
    dropDownPointFileType.Description = 'point file type';
    dropDownPointFileType.Editable = true;
    if isequal(app.extractChessboardPointsParams.pointFileType, 'pcd')
        dropDownPointFileType.Value = 'P';
    elseif isequal(app.extractChessboardPointsParams.pointFileType, 'bin')
        dropDownPointFileType.Value = 'B';
    end
    
    dropDownPointFileType.ValueChangedFcn = @onPointFileTypeChangedCallback;

    dropDownExtrinsicEstimationAlgo = DropDown({'A' 'Co-Mask-LM-G2O'; 'B' 'Co-Mask-GA-LM'; 'C' 'Co-Mask-GA-I-Refine'});
    dropDownExtrinsicEstimationAlgo.Description = 'ExtrinsicEstimationAlgo';
    dropDownExtrinsicEstimationAlgo.Editable = true;
    if isequal(app.ExtrinsicEstimationAlgo, 'Co-Mask-LM-G2O')
        dropDownExtrinsicEstimationAlgo.Value = 'A';
    elseif isequal(app.ExtrinsicEstimationAlgo, 'Co-Mask-GA-LM')
        dropDownExtrinsicEstimationAlgo.Value = 'B';
    elseif isequal(app.ExtrinsicEstimationAlgo, 'Co-Mask-GA-I-Refine')
        dropDownExtrinsicEstimationAlgo.Value = 'C';
    end
    dropDownExtrinsicEstimationAlgo.ValueChangedFcn = @onExtrinsicEstimationAlgoChangedCallback;

    dropDownPointExtractionAlgo = DropDown({'P' 'diffPlane'});
    dropDownPointExtractionAlgo.Description = 'pointExtractionAlgo';
    dropDownPointExtractionAlgo.Editable = true;
    if isequal(app.chessboardLidarPointsExtractionAlgo, 'diffPlane')
        dropDownPointExtractionAlgo.Value = 'P';
    end
    dropDownPointExtractionAlgo.ValueChangedFcn = @onPointExtractionAlgoChangedCallback;

    labelXLimMin = Label('XLimMin:');
    labelXLimMax = Label('XLimMax:');

    XLimMinEditField = EditField();
    XLimMinEditField.Value = num2str(app.extractChessboardPointsParams.pointcloud_limit(1));
    XLimMinEditField.Description = 'XLimMin';
    XLimMinEditField.ValueChangedFcn =  @onXLimMinChangedCallback;

    XLimMaxEditField = EditField();
    XLimMaxEditField.Value = num2str(app.extractChessboardPointsParams.pointcloud_limit(2));
    XLimMaxEditField.Description = 'XLimMax';
    XLimMaxEditField.ValueChangedFcn =  @onXLimMaxChangedCallback;

    labelYLimMin = Label('YLimMin:');
    labelYLimMax = Label('YLimMax:');

    YLimMinEditField = EditField();
    YLimMinEditField.Value = num2str(app.extractChessboardPointsParams.pointcloud_limit(3));
    YLimMinEditField.Description = 'YLimMin';
    YLimMinEditField.ValueChangedFcn =  @onYLimMinChangedCallback;

    YLimMaxEditField = EditField();
    YLimMaxEditField.Value = num2str(app.extractChessboardPointsParams.pointcloud_limit(4));
    YLimMaxEditField.Description = 'YLimMax';
    YLimMaxEditField.ValueChangedFcn =  @onYLimMaxChangedCallback;

    labelZLimMin = Label('ZLimMin:');
    labelZLimMax = Label('ZLimMax:');

    ZLimMinEditField = EditField();
    ZLimMinEditField.Value = num2str(app.extractChessboardPointsParams.pointcloud_limit(5));
    ZLimMinEditField.Description = 'ZLimMin';
    ZLimMinEditField.ValueChangedFcn =  @onZLimMinChangedCallback;

    ZLimMaxEditField = EditField();
    ZLimMaxEditField.Value = num2str(app.extractChessboardPointsParams.pointcloud_limit(6));
    ZLimMaxEditField.Description = 'ZLimMax';
    ZLimMaxEditField.ValueChangedFcn =  @onZLimMaxChangedCallback;

    % assemble
    add(tab, section);
    add(section, column1);
    add(section, column2);
    add(section, column3);
    add(section, column4);
    add(section, column5);
    add(section, column6);
    add(section, column7);
    add(column1, labelPointFileType);
    add(column1, labelPointChessboardAlogo);
    add(column1, labelExtrinsicExtractionAlogo);
    
    add(column2, dropDownPointFileType);
    add(column2, dropDownPointExtractionAlgo);
    add(column2, dropDownExtrinsicEstimationAlgo);
    add(column3, labelXLimMin);
    add(column3, labelXLimMax);
    add(column3, labelYLimMin);
    add(column4, XLimMinEditField);
    add(column4, XLimMaxEditField);
    add(column4, YLimMinEditField);
    add(column5, labelYLimMax);
    add(column5, labelZLimMin);
    add(column5, labelZLimMax);
    add(column6, YLimMaxEditField);
    add(column6, ZLimMinEditField);
    add(column6, ZLimMaxEditField);

    function onPointFileTypeChangedCallback(src, data)
        value = src.SelectedItem;
        if isequal(value, 'P')
            app.extractChessboardPointsParams.pointFileType = 'pcd';
            app.extractChessboardPointsParams.pointFileType = 'pcd';
        elseif isequal(value, 'B')
            app.extractChessboardPointsParams.pointFileType = 'bin';
            app.extractChessboardPointsParams.pointFileType = 'bin';
        end
    end % onPointFileTypeChangedCallback

    function onExtrinsicEstimationAlgoChangedCallback(src, data)
        value = src.SelectedItem;
        if isequal(value, 'A')
            app.ExtrinsicEstimationAlgo = 'Co-Mask-LM-G2O';
        elseif isequal(value, 'B')
            app.ExtrinsicEstimationAlgo = 'Co-Mask-GA-LM';
        elseif isequal(value, 'C')
            app.ExtrinsicEstimationAlgo = 'Co-Mask-GA-I-Refine';
        end
    end % onExtrinsicEstimationAlgoChangedCallback

    function onPointExtractionAlgoChangedCallback(src, data)
        value = src.SelectedItem;
        if isequal(value, 'P')
            app.chessboardLidarPointsExtractionAlgo = 'diffPlane';
        end
    end % onPointExtractionAlgoChangedCallback

    function onXLimMinChangedCallback(src, data)
        value = str2double(src.Text);
        if value >= -200 & value < app.extractChessboardPointsParams.pointcloud_limit(2)
            app.extractChessboardPointsParams.pointcloud_limit(1) = value;
            app.updatePointView();
        else
            src.Text = num2str(app.extractChessboardPointsParams.pointcloud_limit(1));
        end
    end % onXLimMinChangedCallback

    function onXLimMaxChangedCallback(src, data)
        value = str2double(src.Text);
        if value <= 200 & value > app.extractChessboardPointsParams.pointcloud_limit(1)
            app.extractChessboardPointsParams.pointcloud_limit(2) = value;
            app.updatePointView();
        else
            src.Text = num2str(app.extractChessboardPointsParams.pointcloud_limit(2));
        end
    end % onXLimMaxChangedCallback

    function onYLimMinChangedCallback(src, data)
        value = str2double(src.Text);
        if value >= -200 & value < app.extractChessboardPointsParams.pointcloud_limit(4)
            app.extractChessboardPointsParams.pointcloud_limit(3) = value;
            app.updatePointView();
        else
            src.Text = num2str(app.extractChessboardPointsParams.pointcloud_limit(3));
        end
    end % onYLimMinChangedCallback

    function onYLimMaxChangedCallback(src, data)
        value = str2double(src.Text);
        if value <= 200 & value > app.extractChessboardPointsParams.pointcloud_limit(3)
            app.extractChessboardPointsParams.pointcloud_limit(4) = value;
            app.updatePointView();
        else
            src.Text = num2str(app.extractChessboardPointsParams.pointcloud_limit(4));
        end
    end % onYLimMaxChangedCallback

    function onZLimMinChangedCallback(src, data)
        value = str2double(src.Text);
        if value >= -200 & value < app.extractChessboardPointsParams.pointcloud_limit(6)
            app.extractChessboardPointsParams.pointcloud_limit(5) = value;
            app.updatePointView();
        else
            src.Text = num2str(app.extractChessboardPointsParams.pointcloud_limit(5));
        end
    end % onZLimMinChangedCallback

    function onZLimMaxChangedCallback(src, data)
        value = str2double(src.Text);
        if value <= 200 & value > app.extractChessboardPointsParams.pointcloud_limit(5)
            app.extractChessboardPointsParams.pointcloud_limit(6) = value;
            app.updatePointView();
        else
            src.Text = num2str(app.extractChessboardPointsParams.pointcloud_limit(6));
        end
    end % onZLimMaxChangedCallback

end % createExtrinsicOption

function createCalibration(tab, app)
    import matlab.ui.internal.toolstrip.*;
    % create section
    section = Section('Calibration');
    section.Tag = 'CalibrationSection';
    % create column
    column1 = Column();
    column2 = Column();
    column3 = Column();
    column4 = Column();

    % built-in: see matlab.ui.internal.toolstrip.Icon.showStandardIcons()
    ExtractButton = Button('Extract', Icon.RUN_24);
    ExtractButton.Tag = 'extract';
    ExtractButton.Description = 'extract';
    ExtractButton.ButtonPushedFcn = @onExtract;

    % built-in: see matlab.ui.internal.toolstrip.Icon.showStandardIcons()
    CalibrateExtrinsicButton = Button(sprintf('Calibrate\nExtrinsic'), Icon.RUN_24);
    CalibrateExtrinsicButton.Tag = 'calibrateExtrinsic';
    CalibrateExtrinsicButton.Description = 'calibrateExtrinsic';
    CalibrateExtrinsicButton.ButtonPushedFcn = @onCalibrateExtrinsic;

    % built-in: see matlab.ui.internal.toolstrip.Icon.showStandardIcons()
    exportCalibrationParametersButton = SplitButton(sprintf('Export Calibration\nParameters'), Icon.CONFIRM_24);
    exportCalibrationParametersButton.Tag = 'exportCalibrationParametersButton';
    exportCalibrationParametersButton.Description = 'export calibration parameters';
    exportCalibrationParametersButton.DynamicPopupFcn = @(x,y) buildDynamicexportCalibrationParametersPopupList_SmallIcon(app, 'swing');

    % assemble
    add(tab, section);
    add(section, column1);
    add(column1, ExtractButton);
    add(section, column2);
    add(column2, CalibrateExtrinsicButton);
    add(section, column3);
    add(column3, exportCalibrationParametersButton);

    function onExtract(src, data)
        if isempty(app.imagePointPairsStruct)
            warningNoneImagePointDataDialog();
            return;
        end
        % detect the chessboard in images
        patternSize = (app.extractChessboardPointsParams.chessboardProperties(1) - 1) * ...
                        (app.extractChessboardPointsParams.chessboardProperties(2) - 1);
        [imagePointsConsistency, boardSizeConsistency, imageUsedConsistency, imagePoints, boardSize, imageUsed] = detectImageChessboardPoints(  app.imagePointPairsStruct.rawImagesFilename, ...
                                                                                                                                                'matlab', ...
                                                                                                                                                patternSize);

        % detect the chessboard in points
        [lidarPoints, pointsNormal, boardSize, lidarUsed] = detectLidarChessboardPoints(app.imagePointPairsStruct.rawPointsFilename, ...
                                                                                        app.chessboardLidarPointsExtractionAlgo, ...
                                                                                        app.extractChessboardPointsParams);

        % extract the final matching pairs
        matchedIndex = matchValidPair(imageUsedConsistency, lidarUsed);

        % save the data for intrinsic calibration
        imagePoints_ = imagePoints(matchedIndex);

        for i = 1 : length(imagePoints_)
            app.cameraIntrinsicCalibrationData.imagePoints = cat(3, app.cameraIntrinsicCalibrationData.imagePoints, imagePoints_{i});
        end
        app.cameraIntrinsicCalibrationData.worldPoints = generateCheckerboardPoints(boardSizeConsistency, app.extractChessboardPointsParams.chessboardProperties(3));
        imageFileNames = app.imagePointPairsStruct.rawImagesFilename(matchedIndex);
        if isempty(imageFileNames)
            warningNoneImagePointDataDialog();
            return;
        end
        originalImage = imread(imageFileNames{1});
        [mrows, ncols, ~] = size(originalImage);
        app.cameraIntrinsicCalibrationData.ImageSize = [mrows ncols];

        % save the data for extrinsic calibration
        app.cameraLidarextrinsicCalibrationData.imagePoints = imagePoints(matchedIndex);
        app.cameraLidarextrinsicCalibrationData.chessboardSize = boardSize;
        app.cameraLidarextrinsicCalibrationData.imageFilenames = app.imagePointPairsStruct.rawImagesFilename(matchedIndex);
        app.cameraLidarextrinsicCalibrationData.lidarPoints = lidarPoints(matchedIndex);
        app.cameraLidarextrinsicCalibrationData.lidarPointsNormals = pointsNormal(matchedIndex);
        app.cameraLidarextrinsicCalibrationData.lidarFilenames = app.imagePointPairsStruct.rawPointsFilename(matchedIndex);

        app.cameraLidarextrinsicCalibrationData.enable = cell(length(app.cameraLidarextrinsicCalibrationData.imagePoints), 1);
        for i = 1 : length(app.cameraLidarextrinsicCalibrationData.enable)
            app.cameraLidarextrinsicCalibrationData.enable{i} = true;
        end

        app.current_imageFilenames = app.cameraLidarextrinsicCalibrationData.imageFilenames;
        app.current_pointFilenames = app.cameraLidarextrinsicCalibrationData.lidarFilenames;
        app.current_enables = app.cameraLidarextrinsicCalibrationData.enable;

        % update the current_imageSize
        app.current_imageSize = size(imread(app.current_imageFilenames{1}));

        % refresh the content of the browser table
        app.refreshBrowserTableData();

        app.current_index = 1;

        app.updateImagePoint();
        % refresh the camera image chessboard points
        app.updateImageView();
        % refresh the lidar point chessboard points
        app.updatePointView();

        % refresh the image points table view
        app.refreshImagePointsTableData();

        % refresh the lidar points table view
        app.refreshLidarPointsTableData();

        % extract the mask
        onExtractPlaneMask(src, data);

        function warningNoneImagePointDataDialog
            str = sprintf('None Image-Point Data Found! \r\nLoad the image-point data first');
            uiwait(warndlg(str, 'None Image-Point Data'));
        end % warningNoneImagePointDataDialog
    end % onExtract

    function onExtractPlaneMask(src, data)
        if isempty(app.cameraLidarextrinsicCalibrationData.imagePoints)
            warningNoneExtractionDialog();
            return;
        end

        if isempty(app.cameraIntrinsicCalibrationResult.cameraParams)
            warningNoneIntrinsicDialog();
            return;
        end

        current_pair_length = length(app.cameraLidarextrinsicCalibrationData.imagePoints);

        app.cameraLidarextrinsicCalibrationData.chessboardMask = cell(current_pair_length, 1);
        
        for i = 1 : current_pair_length
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            pattern_size = [app.extractChessboardPointsParams.chessboardProperties(1)-1 app.extractChessboardPointsParams.chessboardProperties(2)-1];
            image_size = app.cameraIntrinsicCalibrationResult.cameraParams.ImageSize;
            chessboard_points = app.cameraLidarextrinsicCalibrationData.imagePoints{i};
            app.cameraLidarextrinsicCalibrationData.chessboardMask{i} = generateMaskFromCorners(chessboard_points, image_size, pattern_size);
        end

        function warningNoneExtractionDialog
            str = sprintf('No extraction data found');
            uiwait(warndlg(str, 'Do extraction first'));
        end % warningNoneExtractionDialog

        function warningNoneIntrinsicDialog
            str = sprintf('No intrinsci data found');
            uiwait(warndlg(str, 'Do intrinsic calibration or load first'));
        end % warningNoneExtractionDialog
    end % onExtractPlaneMask

    function onCalibrateExtrinsic(src, data)
        if isempty(app.cameraLidarextrinsicCalibrationData.imagePoints)
            warningNoExtractionDataDialog();
            return;
        end
        % start to calibrate, set the cursor to busy status
        set(app.guiBrowserTable.handle, 'Pointer', 'watch');
        set(app.guiImageView.handle, 'Pointer', 'watch');
        set(app.guiPointView.handle, 'Pointer', 'watch');
        set(app.guiImageDataView.handle, 'Pointer', 'watch');
        set(app.guiPointDataView.handle, 'Pointer', 'watch');
        drawnow;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        estimateExtrinsicCoMask();

        % update the current_camera2lidar_6dof
        if ~isempty(app.cameraLidarextrinsicCalibrationResult.extrinsicMat)
            CameraExtrinsicMat = app.cameraLidarextrinsicCalibrationResult.extrinsicMat;
            R_c2l = rotm2eul(CameraExtrinsicMat(1:3, 1:3));
            T_c2l = CameraExtrinsicMat(1:3, 4);
            app.current_camera2lidar_6dof(1) = R_c2l(1);
            app.current_camera2lidar_6dof(2) = R_c2l(2);
            app.current_camera2lidar_6dof(3) = R_c2l(3);
            app.current_camera2lidar_6dof(4) = T_c2l(1);
            app.current_camera2lidar_6dof(5) = T_c2l(2);
            app.current_camera2lidar_6dof(6) = T_c2l(3);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % finish calibrating, set the cursor to arrow
        set(app.guiBrowserTable.handle, 'Pointer', 'arrow');
        set(app.guiImageView.handle, 'Pointer', 'arrow');
        set(app.guiPointView.handle, 'Pointer', 'arrow');
        set(app.guiImageDataView.handle, 'Pointer', 'arrow');
        set(app.guiPointDataView.handle, 'Pointer', 'arrow');
        drawnow;

        warningCalibrationDoneDialog();

        app.refreshCalibrationResultsTableData();

        function [wpConvHull] = computeConvexHull(worldPoints)
            x = worldPoints(:, 1);
            y = worldPoints(:, 2);

            k = convhull(x, y, 'simplify', true);
            
            % pad with zeros so that Z = 0
            wpConvHull = [x(k), y(k), zeros(length(k), 1)]';
        end

        function estimateExtrinsicCoMask
            if isempty(app.cameraLidarextrinsicCalibrationData.chessboardMask)
                warningNoChessboardMaskDataDialog();
                return;
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % do extrinsic calibration
            % re-select the data according to the enables
            chessboardLidarPoints_ = [];
            chessboardPlaneMaskDT_ = [];
            k = 1;
            for i = 1 : length(app.cameraLidarextrinsicCalibrationData.enable)
                if app.cameraLidarextrinsicCalibrationData.enable{i}
                    chessboardLidarPoints_{k} = app.cameraLidarextrinsicCalibrationData.lidarPoints{i}(:, 1:3);
                    [chessboardPlaneMaskDT_{k}, ~] = bwdist(app.cameraLidarextrinsicCalibrationData.chessboardMask{i});
                    k = k + 1;
                end
            end
            % calibrate the camera and the lidar
            estimateExtrinsicParams = struct();
            estimateExtrinsicParams.initExtrinsicParams = [app.current_camera2lidar_6dof(1) ... 
                                                        app.current_camera2lidar_6dof(2) ...
                                                        app.current_camera2lidar_6dof(3) ...
                                                        app.current_camera2lidar_6dof(4) ...
                                                        app.current_camera2lidar_6dof(5) ...
                                                        app.current_camera2lidar_6dof(6)];

            % save the optimization process 
            estimateExtrinsicOpti_in_process = {};

            if isequal(app.ExtrinsicEstimationAlgo, 'Co-Mask-LM-G2O')
                % replace the init params with the lidar2camera se3
                initExtrinsicParamsLidar2CameraEuler = eulerInv(estimateExtrinsicParams.initExtrinsicParams');
                initExtrinsicParamsLidar2CameraSe3 = euler2se3(initExtrinsicParamsLidar2CameraEuler);

                estimateExtrinsicParams.initExtrinsicParams = initExtrinsicParamsLidar2CameraSe3;

                estimateExtrinsicParams.verbose = true;
                estimateExtrinsicParams.maxIters = 10;
                [Params, estimationErrors, ~, ~, ~, ~] = estimateExtrinsicParametersCoMask_LM_G2O(chessboardLidarPoints_, ...
                                                                    chessboardPlaneMaskDT_, ...
                                                                    app.cameraIntrinsicCalibrationResult.cameraParams, ...
                                                                    estimateExtrinsicParams);
                estimateExtrinsicOpti_in_process = {};
            elseif isequal(app.ExtrinsicEstimationAlgo, 'Co-Mask-GA-LM')
                % replace the init params with the lidar2camera se3
                initExtrinsicParamsLidar2CameraEuler = eulerInv(estimateExtrinsicParams.initExtrinsicParams');
                initExtrinsicParamsLidar2CameraSe3 = euler2se3(initExtrinsicParamsLidar2CameraEuler);

                estimateExtrinsicParams.initExtrinsicParams = initExtrinsicParamsLidar2CameraSe3;

                % first apply GA to find the correct parameters
                estimateExtrinsicParams.loss_type = 'euler';
                estimateExtrinsicParams.searchSpace = [pi, pi, pi, 0, 0, 0]';
                estimateExtrinsicParams.loss_threshold = 500;

                [Params, estimationErrors, ~, ~, ~, ~] = estimateExtrinsicParametersCoMask_GA(chessboardLidarPoints_, ...
                                                                                    chessboardPlaneMaskDT_, ...
                                                                                    app.cameraIntrinsicCalibrationResult.cameraParams, ...
                                                                                    estimateExtrinsicParams);
                R_c2l = rotm2eul(Params.extrinsicMat(1:3, 1:3));
                T_c2l = Params.extrinsicMat(1:3, 4);
                estimateExtrinsicParams.initExtrinsicParams = [
                    R_c2l(1) ...
                    R_c2l(2) ...
                    R_c2l(3) ...
                    T_c2l(1) ...
                    T_c2l(2) ...
                    T_c2l(3) ...
                ];
                fprintf('GA reached the threshold [%.4f/%.4f], parameters: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', ...
                            estimationErrors, ...
                            estimateExtrinsicParams.loss_threshold, ...
                            estimateExtrinsicParams.initExtrinsicParams(1), ...
                            estimateExtrinsicParams.initExtrinsicParams(2), ...
                            estimateExtrinsicParams.initExtrinsicParams(3), ...
                            estimateExtrinsicParams.initExtrinsicParams(4), ...
                            estimateExtrinsicParams.initExtrinsicParams(5), ...
                            estimateExtrinsicParams.initExtrinsicParams(6) ...
                            );
                initExtrinsicParamsLidar2CameraEuler = eulerInv(estimateExtrinsicParams.initExtrinsicParams');
                initExtrinsicParamsLidar2CameraSe3 = euler2se3(initExtrinsicParamsLidar2CameraEuler);

                estimateExtrinsicParams.initExtrinsicParams = initExtrinsicParamsLidar2CameraSe3;
                % then apply the LM process

                estimateExtrinsicParams.verbose = true;
                estimateExtrinsicParams.maxIters = 10;
                [Params, estimationErrors, ~, ~, ~, ~] = estimateExtrinsicParametersCoMask_LM_G2O(chessboardLidarPoints_, ...
                                                                    chessboardPlaneMaskDT_, ...
                                                                    app.cameraIntrinsicCalibrationResult.cameraParams, ...
                                                                    estimateExtrinsicParams);
                estimateExtrinsicOpti_in_process = {};
            elseif isequal(app.ExtrinsicEstimationAlgo, 'Co-Mask-GA-I-Refine')
                estimateExtrinsicParams = struct();
                % get the init extrinsic parameters, [yaw pitch roll x y z]
                initExtrinsicParams = [
                  app.current_camera2lidar_6dof(1) ...
                  app.current_camera2lidar_6dof(2) ...
                  app.current_camera2lidar_6dof(3) ...
                  app.current_camera2lidar_6dof(4) ...
                  app.current_camera2lidar_6dof(5) ...
                  app.current_camera2lidar_6dof(6) ...
                ];
                % get the init intrinsic parameters, [fx fy cx cy k1 k2]
                K = app.cameraIntrinsicCalibrationResult.cameraParams.IntrinsicMatrix';
                initIntrinsicParams = [
                  K(1, 1) ...
                  K(2, 2) ...
                  K(1, 3) ...
                  K(2, 3) ...
                  app.cameraIntrinsicCalibrationResult.cameraParams.RadialDistortion(1) ...
                  app.cameraIntrinsicCalibrationResult.cameraParams.RadialDistortion(2) ...
                ];
                estimateExtrinsicParams.initIntrinsicExtrinsicParams = [initExtrinsicParams initIntrinsicParams];
                estimateExtrinsicParams.searchSpace = app.intrinsic_extrinsic_refine_search_space;

                [Params, estimationErrors] = estimateIntrinsicExtrinsicParametersCoMask_GA(chessboardLidarPoints_, ...
                                                                                    chessboardPlaneMaskDT_, ...
                                                                                    estimateExtrinsicParams);

                % update the intrinsic parameters
                estimated_IntrinsicMatrix = [Params.fx 0 Params.cx; 0 Params.fy Params.cy; 0 0 1]';
                estimated_RadialDistortion = [Params.k1 Params.k2];
                estiamted_TangentialDistortion = [0 0];
                cameraParams = cameraParameters('IntrinsicMatrix', estimated_IntrinsicMatrix, ...
                                'RadialDistortion', estimated_RadialDistortion, ...
                                'TangentialDistortion', estiamted_TangentialDistortion, ...
                                'ImageSize', [app.cameraIntrinsicCalibrationResult.cameraParams.ImageSize(1) app.cameraIntrinsicCalibrationResult.cameraParams.ImageSize(2)]);
                app.cameraIntrinsicCalibrationResult.cameraParams = cameraParams;

                estimateExtrinsicOpti_in_process = {};
            end
            
            % 3.1408 0.0195 -1.6417 0.1020 -0.5123 -0.4943
            % 0 0 -pi/2 0 0 0
            % pi 0 -pi/2 0 0 0
            if ~isempty(estimationErrors)
                % update the cameraLidarextrinsicCalibrationResult
                app.cameraLidarextrinsicCalibrationResult.extrinsicMat = Params.extrinsicMat;
                app.cameraLidarextrinsicCalibrationResult.estimationErrors = estimationErrors;
                app.cameraLidarextrinsicCalibrationResult.opti_in_process = estimateExtrinsicOpti_in_process;
                app.refreshCalibrationResultsTableData();

                % update the cameraLidarextrinsicCalibrationParams
                app.cameraLidarextrinsicCalibrationParams = estimateExtrinsicParams;
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            function warningNoChessboardMaskDataDialog
                str = sprintf('No Chessboard Mask Found');
                uiwait(warndlg(str, ' Do plane mask extraction first'));
            end % warningNoChessboardMaskDataDialog
        end %estimateExtrinsicCoMask

        function warningCalibrationDoneDialog
            str = sprintf('Calibration Done!');
            uiwait(warndlg(str, ' Calibration Done'));
        end % warningNoneCalibrationDialog

        function warningNoExtractionDataDialog
            str = sprintf('No Extraction Data!');
            uiwait(warndlg(str, ' Do Extraction befor calibration'));
        end % warningNoExtractionDataDialog

        function warningNoIntrinsicDataDialog
            str = sprintf('No Intrinsic Data!');
            uiwait(warndlg(str, ' Do Intrinsic Calibration befor Extrinsic Calibration'));
        end % warningNoIntrinsicDataDialog
    end % onCalibrateExtrinsic

    function popup = buildDynamicexportCalibrationParametersPopupList_SmallIcon(app, mode)
        import matlab.ui.internal.toolstrip.*;
        popup = PopupList();
        item1 = ListItem('extrinsic yml', Icon(javaObjectEDT('javax.swing.ImageIcon', fullfile(app.current_dir, 'icons', 'export_autoware_24.png'))));
        item1.Description = 'extrinsic autoware style';
        item1.ShowDescription = false;
        item1.ItemPushedFcn = @(x, y) exportAutowareYamlExtrinsic(x, y, app);
        popup.add(item1);

        item2 = ListItem('calib data mat', Icon.MATLAB_24);
        item2.Description = 'calib data mat';
        item2.ShowDescription = false;
        item2.ItemPushedFcn = @(x, y) exportCalibDataMat(x, y, app);
        popup.add(item2);

        function exportAutowareYamlExtrinsic(src, data, app)
            % check if the intrinsic parameters exist
            if ~isempty(app.cameraIntrinsicCalibrationResult.cameraParams)
                K = (app.cameraIntrinsicCalibrationResult.cameraParams.IntrinsicMatrix)';
                if isequal(app.cameraIntrinsicCalibrationData.NumRadialDistortionCoefficients, 3)
                    DistCoeff = [   app.cameraIntrinsicCalibrationResult.cameraParams.RadialDistortion(1) ...
                                    app.cameraIntrinsicCalibrationResult.cameraParams.RadialDistortion(2) ...
                                    app.cameraIntrinsicCalibrationResult.cameraParams.RadialDistortion(3) ...
                                    app.cameraIntrinsicCalibrationResult.cameraParams.TangentialDistortion(1) ...
                                    app.cameraIntrinsicCalibrationResult.cameraParams.TangentialDistortion(2)];
                else
                    DistCoeff = [   app.cameraIntrinsicCalibrationResult.cameraParams.RadialDistortion(1) ...
                                    app.cameraIntrinsicCalibrationResult.cameraParams.RadialDistortion(2) ...
                                    app.cameraIntrinsicCalibrationResult.cameraParams.TangentialDistortion(1) ...
                                    app.cameraIntrinsicCalibrationResult.cameraParams.TangentialDistortion(2)];
                end
                imageSize = [   app.cameraIntrinsicCalibrationResult.cameraParams.ImageSize(1) ...
                                app.cameraIntrinsicCalibrationResult.cameraParams.ImageSize(2)];
            else
                warningNoneIntrinsicCalibrationDialog();
                return;
            end

            % check if the extrinsic parameters exist
            % if ~isempty(app.cameraLidarextrinsicCalibrationResult.estimationErrors)
            %     CameraExtrinsicMat = app.cameraLidarextrinsicCalibrationResult.extrinsicMat;
            %     ReprojectionError = app.cameraLidarextrinsicCalibrationResult.estimationErrors;
            % else
            %     warningNoneExtrinsicCalibrationDialog();
            % end


            CameraExtrinsicMat = eye(4);
            CameraExtrinsicMat(1:3, 1:3) = eul2rotm([app.current_camera2lidar_6dof(1) ...
                                                    app.current_camera2lidar_6dof(2) ...
                                                    app.current_camera2lidar_6dof(3)]);
            CameraExtrinsicMat(1:3, 4) = [app.current_camera2lidar_6dof(4) ...
                                            app.current_camera2lidar_6dof(5) ...
                                            app.current_camera2lidar_6dof(6)]';
            ReprojectionError = -1;

            % choose a folder to save the configuration file
            [FileName, PathName] = uiputfile('test.yml', 'save calibration parameters file', [app.current_dir, '/test.yml']);
            if isequal(FileName, 0)
                disp(myLog('user selected cancel'));
                return;
            else
                disp(myLog(['user selected ', [PathName FileName]]));
            end
            exportAutowareStyleYmlExtrinsic();
            

            function warningNoneIntrinsicCalibrationDialog
                str = sprintf('None Intrinsic Calibration Data Found! \r\nDo intrinsic calibration first!');
                uiwait(warndlg(str, 'None Intrinsic Calibration'));
            end % warningNoneIntrinsicCalibrationDialog

            function warningNoneExtrinsicCalibrationDialog
                str = sprintf('None Extrinsic Calibration Data Found! \r\nDo extrinsic calibration first!');
                uiwait(warndlg(str, 'None Extrinsic Calibration'));
            end % warningNoneExtrinsicCalibrationDialog

            function warningCalibrationFileDoneDialog
                str = sprintf('Export Calibration File Done! \r\n');
                uiwait(warndlg(str, 'Export Calibration File Done'));
            end % warningCalibrationFileDoneDialog

            function exportAutowareStyleYmlExtrinsic
                % export the parameters to the yaml file
                fid = fopen([PathName FileName], 'w');
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                str = sprintf('CameraExtrinsicMat:\r\n');
                fprintf(fid, str);
                str = sprintf('  rows: %d\r\n', size(CameraExtrinsicMat, 1));
                fprintf(fid, str);
                str = sprintf('  cols: %d\r\n', size(CameraExtrinsicMat, 2));
                fprintf(fid, str);
                str = sprintf('  dt: d\r\n');
                fprintf(fid, str);
                str = sprintf('  data: ');
                fprintf(fid, str);
                str = sprintf('[ ');
                for i = 1 : size(CameraExtrinsicMat, 1)
                    for j = 1 : size(CameraExtrinsicMat, 2)
                        str = sprintf('%s%.6f', str, CameraExtrinsicMat(i, j));
                        if ~(j == size(CameraExtrinsicMat, 2) && i == size(CameraExtrinsicMat, 1))
                            str = sprintf('%s, ', str);
                        end
                    end
                    if i ~= size(CameraExtrinsicMat, 1)
                        str = sprintf('%s\r\n          ', str);
                    else
                        str = sprintf('%s]\r\n', str);
                    end
                end
                fprintf(fid, str);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                str = sprintf('CameraMat:\r\n');
                fprintf(fid, str);
                str = sprintf('  rows: %d\r\n', size(K, 1));
                fprintf(fid, str);
                str = sprintf('  cols: %d\r\n', size(K, 2));
                fprintf(fid, str);
                str = sprintf('  dt: d\r\n');
                fprintf(fid, str);
                str = sprintf('  data: ');
                fprintf(fid, str);
                str = sprintf('[ ');
                for i = 1 : size(K, 1)
                    for j = 1 : size(K, 2)
                        str = sprintf('%s%.6f', str, K(i, j));
                        if ~(j == size(K, 2) && i == size(K, 1))
                            str = sprintf('%s, ', str);
                        end
                    end
                    if i ~= size(K, 1)
                        str = sprintf('%s\r\n          ', str);
                    else
                        str = sprintf('%s]\r\n', str);
                    end
                end
                fprintf(fid, str);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                str = sprintf('DistCoeff:\r\n');
                fprintf(fid, str);
                str = sprintf('  rows: %d\r\n', size(DistCoeff, 1));
                fprintf(fid, str);
                str = sprintf('  cols: %d\r\n', size(DistCoeff, 2));
                fprintf(fid, str);
                str = sprintf('  dt: d\r\n');
                fprintf(fid, str);
                str = sprintf('  data: ');
                fprintf(fid, str);
                str = sprintf('[ ');
                for i = 1 : size(DistCoeff, 1)
                    for j = 1 : size(DistCoeff, 2)
                        str = sprintf('%s%.6f', str, DistCoeff(i, j));
                        if ~(j == size(DistCoeff, 2) && i == size(DistCoeff, 1))
                            str = sprintf('%s, ', str);
                        end
                    end
                    if i ~= size(DistCoeff, 1)
                        str = sprintf('%s\r\n          ', str);
                    else
                        str = sprintf('%s]\r\n', str);
                    end
                end
                fprintf(fid, str);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                str = sprintf('ImageSize: [ %d, %d ]\r\n', imageSize(1), imageSize(2));
                fprintf(fid, str);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                str = sprintf('ReprojectionError: %.6f\r\n', ReprojectionError);
                fprintf(fid, str);
                fclose(fid);

                warningCalibrationFileDoneDialog();
            end

        end % exportAutowareStyleYmlExtrinsic

        function exportCalibDataMat(src, data, app)
            % check if the intrinsic parameters exist
            saved_mat = struct();
            saved_mat.current_images_dir = app.current_images_dir;
            saved_mat.current_points_dir = app.current_points_dir;
            saved_mat.extractChessboardPointsParams = app.extractChessboardPointsParams;
            saved_mat.imagePointPairsStruct = app.imagePointPairsStruct;
            saved_mat.cameraIntrinsicCalibrationData = app.cameraIntrinsicCalibrationData;
            saved_mat.cameraIntrinsicCalibrationResult = app.cameraIntrinsicCalibrationResult;
            saved_mat.cameraLidarextrinsicCalibrationData = app.cameraLidarextrinsicCalibrationData;
            saved_mat.cameraLidarextrinsicCalibrationResult = app.cameraLidarextrinsicCalibrationResult;
            saved_mat.current_imageFilenames = app.current_imageFilenames;
            saved_mat.current_pointFilenames = app.current_pointFilenames;
            saved_mat.current_camera2lidar_6dof = app.current_camera2lidar_6dof;

            saved_mat.cameraLidarextrinsicCalibrationParams = app.cameraLidarextrinsicCalibrationParams;
            saved_mat.ExtrinsicEstimationAlgo = app.ExtrinsicEstimationAlgo;
            saved_mat.chessboardLidarPointsExtractionAlgo = app.chessboardLidarPointsExtractionAlgo;

            % choose a folder to save the configuration file
            [FileName, PathName] = uiputfile('test.mat', 'save calibration data mat file', [app.current_dir, '/test.mat']);
            if isequal(FileName, 0)
                disp(myLog('user selected cancel'));
                return;
            else
                disp(myLog(['user selected ', [PathName FileName]]));
            end

            saved_mat_file = [PathName FileName];
            save(saved_mat_file, 'saved_mat');

        end % exportCalibDataMat
    end

end % createExtrinsicCalibration

function createViewControl(tab, app)
    import matlab.ui.internal.toolstrip.*
    % create section
    section = Section('Control');
    section.Tag = 'controlSection';
    % create column
    column1 = Column();
    column2 = Column();
    column3 = Column('Width', 120, 'HorizontalAlignment', 'center');
    column4 = Column('Width', 60, 'HorizontalAlignment', 'center');
    column5 = Column('Width', 120, 'HorizontalAlignment', 'center');
    column6 = Column();
    column7 = Column('Width', 60, 'HorizontalAlignment', 'center');
    column8 = Column();
    column9 = Column();
    column10 = Column();
    column11 = Column('Width', 60, 'HorizontalAlignment', 'center');
    column12 = Column();
    column13 = Column();
    column14 = Column();
    column15 = Column('Width', 120, 'HorizontalAlignment', 'center');

    %%%%%%%%%%%%%%%% add control tool for image and point view
    %% button
    % add pointer button
    % built-in: see Icon.showStandardIcons()
    imageFile = fullfile(app.current_dir, 'icons', 'arrow_16.png');
    pointerButton = Button('Pointer', Icon(javaObjectEDT('javax.swing.ImageIcon',imageFile)));
    pointerButton.Tag = 'pointerButton';
    pointerButton.Description = 'pointer';
    pointerButton.ButtonPushedFcn = @onPointer;

    % add select button
    % built-in: see Icon.showStandardIcons()
    imageFile = fullfile(app.current_dir, 'icons', 'cross_16.png');
    selectButton = Button('Select', Icon(javaObjectEDT('javax.swing.ImageIcon',imageFile)));
    selectButton.Tag = 'selectButton';
    selectButton.Description = 'select';
    selectButton.ButtonPushedFcn = @onSelect;
    % add data cursor button
    % built-in: see Icon.showStandardIcons()
    imageFile = fullfile(app.current_dir, 'icons', 'cursor_16.png');
    dataCursorButton = Button('Cursor', Icon(javaObjectEDT('javax.swing.ImageIcon',imageFile)));
    dataCursorButton.Tag = 'dataCursorButton';
    dataCursorButton.Description = 'dataCursor';
    dataCursorButton.ButtonPushedFcn = @onDataCursor;

    % add move button
    % built-in: see Icon.showStandardIcons()
    imageFile = fullfile(app.current_dir, 'icons', 'hand_16.png');
    moveButton = Button('Move', Icon(javaObjectEDT('javax.swing.ImageIcon',imageFile)));
    moveButton.Tag = 'moveButton';
    moveButton.Description = 'move';
    moveButton.ButtonPushedFcn = @onMove;

     % add zoom button
    % built-in: see Icon.showStandardIcons()
    imageFile = fullfile(app.current_dir, 'icons', 'zoom_out_16.png');
    zoomButton = Button('Zoom', Icon(javaObjectEDT('javax.swing.ImageIcon',imageFile)));
    zoomButton.Tag = 'zoomButton';
    zoomButton.Description = 'zoom';
    zoomButton.ButtonPushedFcn = @onZoom;

     % add rotate button
    % built-in: see Icon.showStandardIcons()
    imageFile = fullfile(app.current_dir, 'icons', 'rotate_16.png');
    rotateButton = Button('Rotate', Icon(javaObjectEDT('javax.swing.ImageIcon',imageFile)));
    rotateButton.Tag = 'rotateButton';
    rotateButton.Description = 'rotate';
    rotateButton.ButtonPushedFcn = @onRotate;

    % add render type
    labelPointRenderType = Label('point render type');

    pointRenderType = DropDown({'P' 'plain';'H' 'height';'R' 'range';'I' 'intensity'});
    pointRenderType.Description = 'point render type';
    pointRenderType.Editable = true;
    if isequal(app.current_point_render_type, 'plain')
        pointRenderType.Value = 'P';
    elseif isequal(app.current_point_render_type, 'height')
        pointRenderType.Value = 'H';
    elseif isequal(app.current_point_render_type, 'range')
        pointRenderType.Value = 'R';
    elseif isequal(app.current_point_render_type, 'intensity')
        pointRenderType.Value = 'I';
    end
    pointRenderType.ValueChangedFcn = @onPointRenderTypeChangedCallback;

    % add point view control parameter
    labelPointSize = Label('point size');

    pointSizeSlider = Slider([1 10],1);
    pointSizeSlider.Steps = app.current_point_size;

    pointSizeEditField = EditField();
    pointSizeEditField.Value = num2str(app.current_point_size);
    pointSizeEditField.Description = 'point size';

    pointSizeSlider.ValueChangedFcn = @(x, y) onPointSizeChangedCallback(x, y, app, pointSizeEditField);
    pointSizeEditField.ValueChangedFcn = @(x, y) onPointSizeEditChangedCallback(x, y, app, pointSizeSlider);

    % add render type
    labelImageRenderType = Label('image render type');

    imageRenderType = DropDown({'P' 'plain';'H' 'height';'R' 'range';'I' 'intensity'});
    imageRenderType.Description = 'image render type';
    imageRenderType.Editable = true;
    if isequal(app.current_image_render_type, 'plain')
        imageRenderType.Value = 'P';
    elseif isequal(app.current_image_render_type, 'height')
        imageRenderType.Value = 'H';
    elseif isequal(app.current_image_render_type, 'range')
        imageRenderType.Value = 'R';
    elseif isequal(app.current_image_render_type, 'intensity')
        imageRenderType.Value = 'I';
    end
    imageRenderType.ValueChangedFcn = @onImageRenderTypeChangedCallback;

    labelRotZ = Label('RotZ:');
    labelRotY = Label('RotY:');
    labelRotX = Label('RotX:');
    labelVecX = Label('VecX:');
    labelVecY = Label('VecY:');
    labelVecZ = Label('VecZ:');

    RotZEditField = EditField();
    RotZEditField.Value = num2str(app.current_camera2lidar_6dof(1));
    RotZEditField.Description = 'RotZ';
    RotZEditField.ValueChangedFcn =  @onRotZChangedCallback;

    % built-in: see Icon.showStandardIcons()
    AddRotZButton = Button('', Icon.UP_16);
    AddRotZButton.Tag = 'addRotZ';
    AddRotZButton.Description = 'addRotZ';
    AddRotZButton.ButtonPushedFcn = @(x, y) onAddRotZ(x, y, app, RotZEditField);

    % built-in: see Icon.showStandardIcons()
    SubRotZButton = Button('', Icon.DOWN_16);
    SubRotZButton.Tag = 'subRotZ';
    SubRotZButton.Description = 'subRotZ';
    SubRotZButton.ButtonPushedFcn = @(x, y) onSubRotZ(x, y, app, RotZEditField);

    RotYEditField = EditField();
    RotYEditField.Value = num2str(app.current_camera2lidar_6dof(2));
    RotYEditField.Description = 'RotY';
    RotYEditField.ValueChangedFcn =  @onRotYChangedCallback;

    % built-in: see Icon.showStandardIcons()
    AddRotYButton = Button('', Icon.UP_16);
    AddRotYButton.Tag = 'addRotY';
    AddRotYButton.Description = 'addRotY';
    AddRotYButton.ButtonPushedFcn = @(x, y) onAddRotY(x, y, app, RotYEditField);

    % built-in: see Icon.showStandardIcons()
    SubRotYButton = Button('', Icon.DOWN_16);
    SubRotYButton.Tag = 'subRotY';
    SubRotYButton.Description = 'subRotY';
    SubRotYButton.ButtonPushedFcn = @(x, y) onSubRotY(x, y, app, RotYEditField);

    RotXEditField = EditField();
    RotXEditField.Value = num2str(app.current_camera2lidar_6dof(3));
    RotXEditField.Description = 'RotX';
    RotXEditField.ValueChangedFcn =  @onRotXChangedCallback;

    % built-in: see Icon.showStandardIcons()
    AddRotXButton = Button('', Icon.UP_16);
    AddRotXButton.Tag = 'addRotX';
    AddRotXButton.Description = 'addRotX';
    AddRotXButton.ButtonPushedFcn = @(x, y) onAddRotX(x, y, app, RotXEditField);

    % built-in: see Icon.showStandardIcons()
    SubRotXButton = Button('', Icon.DOWN_16);
    SubRotXButton.Tag = 'subRotX';
    SubRotXButton.Description = 'subRotX';
    SubRotXButton.ButtonPushedFcn = @(x, y) onSubRotX(x, y, app, RotXEditField);

    VecXEditField = EditField();
    VecXEditField.Value = num2str(app.current_camera2lidar_6dof(4));
    VecXEditField.Description = 'VecX';
    VecXEditField.ValueChangedFcn =  @onVecXChangedCallback;

    % built-in: see Icon.showStandardIcons()
    AddVecXButton = Button('', Icon.UP_16);
    AddVecXButton.Tag = 'addVecX';
    AddVecXButton.Description = 'addVecX';
    AddVecXButton.ButtonPushedFcn = @(x, y) onAddVecX(x, y, app, VecXEditField);

    % built-in: see Icon.showStandardIcons()
    SubVecXButton = Button('', Icon.DOWN_16);
    SubVecXButton.Tag = 'subVecX';
    SubVecXButton.Description = 'subVecX';
    SubVecXButton.ButtonPushedFcn = @(x, y) onSubVecX(x, y, app, VecXEditField);

    VecYEditField = EditField();
    VecYEditField.Value = num2str(app.current_camera2lidar_6dof(5));
    VecYEditField.Description = 'VecY';
    VecYEditField.ValueChangedFcn =  @onVecYChangedCallback;

    % built-in: see Icon.showStandardIcons()
    AddVecYButton = Button('', Icon.UP_16);
    AddVecYButton.Tag = 'addVecY';
    AddVecYButton.Description = 'addVecY';
    AddVecYButton.ButtonPushedFcn = @(x, y) onAddVecY(x, y, app, VecYEditField);

    % built-in: see Icon.showStandardIcons()
    SubVecYButton = Button('', Icon.DOWN_16);
    SubVecYButton.Tag = 'subVecY';
    SubVecYButton.Description = 'subVecY';
    SubVecYButton.ButtonPushedFcn = @(x, y) onSubVecY(x, y, app, VecYEditField);

    VecZEditField = EditField();
    VecZEditField.Value = num2str(app.current_camera2lidar_6dof(6));
    VecZEditField.Description = 'VecZ';
    VecZEditField.ValueChangedFcn =  @onVecZChangedCallback;

    % built-in: see Icon.showStandardIcons()
    AddVecZButton = Button('', Icon.UP_16);
    AddVecZButton.Tag = 'addVecZ';
    AddVecZButton.Description = 'addVecZ';
    AddVecZButton.ButtonPushedFcn = @(x, y) onAddVecZ(x, y, app, VecZEditField);

    % built-in: see Icon.showStandardIcons()
    SubVecZButton = Button('', Icon.DOWN_16);
    SubVecZButton.Tag = 'subVecZ';
    SubVecZButton.Description = 'subVecZ';
    SubVecZButton.ButtonPushedFcn = @(x, y) onSubVecZ(x, y, app, VecZEditField);

    % built-in: see Icon.showStandardIcons()
    RefreshExtrinsicButton = Button('Refresh', Icon.REFRESH_24);
    RefreshExtrinsicButton.Tag = 'refresh extrinsic';
    RefreshExtrinsicButton.Description = 'refresh extrinsic';
    RefreshExtrinsicButton.ButtonPushedFcn = @(x, y) onRefreshExtrinsic(x, y, app, RotZEditField, ...
                                                                                    RotYEditField, ...
                                                                                    RotXEditField, ...
                                                                                    VecXEditField, ...
                                                                                    VecYEditField, ...
                                                                                    VecZEditField);

    % add reprojection type
    labelReprojectionType = Label('reprojection type');

    reprojectionType = DropDown({'U' 'undistort';'D' 'distort'});
    reprojectionType.Description = 'reprojection type';
    reprojectionType.Editable = true;
    if isequal(app.current_reprojection_type, 'undistort')
        reprojectionType.Value = 'U';
    elseif isequal(app.current_reprojection_type, 'distort')
        reprojectionType.Value = 'D';
    end
    reprojectionType.ValueChangedFcn = @onReprojectionTypeChangedCallback;

    % assemble
    add(tab, section);
    add(section, column1);
    add(section, column2);
    add(section, column3);
    add(section, column4);
    add(section, column5);

    add(section, column6);
    add(section, column7);
    add(section, column8);
    add(section, column9);
    add(section, column10);
    add(section, column11);
    add(section, column12);
    add(section, column13);
    add(section, column14);
    add(section, column15);

    add(column1, pointerButton);
    add(column1, selectButton);
    add(column1, dataCursorButton);

    add(column2, moveButton);
    add(column2, zoomButton);
    add(column2, rotateButton);

    add(column3, labelPointRenderType);
    add(column3, pointRenderType);

    add(column4, labelPointSize);
    add(column4, pointSizeSlider);
    add(column4, pointSizeEditField);

    add(column5, labelImageRenderType);
    add(column5, imageRenderType);

    add(column6, labelRotZ);
    add(column6, labelRotY);
    add(column6, labelRotX);

    add(column7, RotZEditField);
    add(column7, RotYEditField);
    add(column7, RotXEditField);

    add(column8, AddRotZButton);
    add(column8, AddRotYButton);
    add(column8, AddRotXButton);

    add(column9, SubRotZButton);
    add(column9, SubRotYButton);
    add(column9, SubRotXButton);

    add(column10, labelVecX);
    add(column10, labelVecY);
    add(column10, labelVecZ);

    add(column11, VecXEditField);
    add(column11, VecYEditField);
    add(column11, VecZEditField);

    add(column12, AddVecXButton);
    add(column12, AddVecYButton);
    add(column12, AddVecZButton);

    add(column13, SubVecXButton);
    add(column13, SubVecYButton);
    add(column13, SubVecZButton);

    add(column14, RefreshExtrinsicButton);

    add(column15, labelReprojectionType);
    add(column15, reprojectionType);

    function onPointer(src, data)
        ax = gca;
        if isequal(ax, app.guiImageView.axesImageView)
            set(app.guiImageView.handle, 'Pointer', 'arrow');
            zoom off;
            rotate3d off;
            set(app.dataCursorImageHandle, 'Enable', 'off');
            % delete the datatip
            delete(findall(gcf, 'Type', 'hggroup'));
        elseif isequal(ax, app.guiPointView.axesPointView)
            set(app.guiPointView.handle, 'Pointer', 'arrow');
            zoom off;
            rotate3d off;
            set(app.dataCursorPointHandle, 'Enable', 'off');
        end
        drawnow;
    end  % onPointer

    function onDataCursor(src, data)
        ax = gca;
        if isequal(ax, app.guiImageView.axesImageView)
            zoom off;
            rotate3d off;
            app.dataCursorImageHandle = datacursormode(app.guiImageView.handle);
            app.dataCursorImageHandle.UpdateFcn = @onCursorImageUpdateFcn;
            app.dataCursorImageHandle.Enable = 'on';
        elseif isequal(ax, app.guiPointView.axesPointView)
            zoom off;
            rotate3d off;
            app.dataCursorPointHandle = datacursormode(app.guiPointView.handle);
            app.dataCursorPointHandle.UpdateFcn = @onCursorPointUpdateFcn;
            app.dataCursorPointHandle.DisplayStyle = 'window';
            app.dataCursorPointHandle.Enable = 'on';
        end
        drawnow;
    end  % onDataCursor

    function onSelect(src, data)
        ax = gca;
        if isequal(ax, app.guiImageView.axesImageView)
            set(app.guiImageView.handle, 'Pointer', 'cross');
            zoom off;
            rotate3d off;
            set(app.dataCursorImageHandle, 'Enable', 'off');
            % delete the datatip
            delete(findall(gcf, 'Type', 'hggroup'));
        elseif isequal(ax, app.guiPointView.axesPointView)
            set(app.guiPointView.handle, 'Pointer', 'cross');
            zoom off;
            rotate3d off;
            set(app.dataCursorPointHandle, 'Enable', 'off');
        end
        drawnow;
    end  % onSelect

    function onMove(src, data)
        ax = gca;
        if isequal(ax, app.guiImageView.axesImageView)
            set(app.guiImageView.handle, 'Pointer', 'hand');
            zoom off;
            rotate3d off;
            set(app.dataCursorImageHandle, 'Enable', 'off');
            % delete the datatip
            delete(findall(gcf, 'Type', 'hggroup'));
        elseif isequal(ax, app.guiPointView.axesPointView)
            set(app.guiPointView.handle, 'Pointer', 'hand');
            zoom off;
            rotate3d off;
            set(app.dataCursorPointHandle, 'Enable', 'off');
        end
        drawnow;
    end  % onMove

    function onZoom(src, data)
        ax = gca;
        if isequal(ax, app.guiImageView.axesImageView)
            rotate3d off;
            set(app.dataCursorImageHandle, 'Enable', 'off');
            % delete the datatip
            delete(findall(gcf, 'Type', 'hggroup'));
            % enable zoom
            zoom on;
        elseif isequal(ax, app.guiPointView.axesPointView)
            rotate3d off;
            set(app.dataCursorPointHandle, 'Enable', 'off');
    
            % enable zoom 
            h = zoom;
            h.ActionPostCallback = @onZoomPostCallback;
            h.Enable = 'on';
        end
        drawnow;
        function onZoomPostCallback(obj,evd)
           
        end % onZoomPostCallback
    end  % onZoom

    function onRotate(src, data)
        ax = gca;
        if isequal(ax, app.guiImageView.axesImageView)
            zoom off;
            set(app.dataCursorImageHandle, 'Enable', 'off');
            % delete the datatip
            delete(findall(gcf, 'Type', 'hggroup'));
            rotate3d on;
        elseif isequal(ax, app.guiPointView.axesPointView)
            zoom off;
            set(app.dataCursorPointHandle, 'Enable', 'off');
            h = rotate3d;
            h.ActionPostCallback = @onRotatePostCallback;
            h.Enable = 'on';
        end
        drawnow;
        function onRotatePostCallback(obj,evd)
            newView = round(evd.Axes.View);
            app.current_point_view.az = newView(1);
            app.current_point_view.el = newView(2);
        end % onRotatePostCallback
    end  % onRotate

    function onPointRenderTypeChangedCallback(src, data)
        value = src.SelectedItem;
        if isequal(value, 'P')
            app.current_point_render_type = 'plain';
        elseif isequal(value, 'H')
            app.current_point_render_type = 'height';
        elseif isequal(value, 'R')
            app.current_point_render_type = 'range';
        elseif isequal(value, 'I')
            app.current_point_render_type = 'intensity';
        end
        app.updatePointView();
    end % onPointRenderTypeChangedCallback

    function onPointSizeChangedCallback(src, data, app, pointSizeEditField)
        app.current_point_size = src.Value;
        pointSizeEditField.Value = num2str(src.Value);
        app.updateImageView();
        app.updatePointView();
    end % onPointSizeChangedCallback
    
    function onPointSizeEditChangedCallback(src, data, app, pointSizeSlider)
        value = str2double(src.Text);
        if value >= 1 & value <= 10
            app.current_point_size = floor(value);
            pointSizeSlider.Value = floor(value);
            app.updateImageView();
            app.updatePointView();
        else
            src.Text = num2str(app.current_point_size);
        end
    end % onPointSizeChangedCallback

    function onImageRenderTypeChangedCallback(src, data)
        value = src.SelectedItem;
        if isequal(value, 'P')
            app.current_image_render_type = 'plain';
        elseif isequal(value, 'H')
            app.current_image_render_type = 'height';
        elseif isequal(value, 'R')
            app.current_image_render_type = 'range';
        elseif isequal(value, 'I')
            app.current_image_render_type = 'intensity';
        end
        app.updateImageView();
    end % onImageRenderTypeChangedCallback

    function onRotZChangedCallback(src, data)
        value = str2double(src.Text);
        if value < 3.14 & value > -3.14
            app.current_camera2lidar_6dof(1) = value;
            app.updateImageView();
        else
            src.Text = num2str(app.current_camera2lidar_6dof(1));
        end
    end % onRotZChangedCallback

    function onRotYChangedCallback(src, data)
        value = str2double(src.Text);
        if value < 3.14 & value > -3.14
            app.current_camera2lidar_6dof(2) = value;
            app.updateImageView();
        else
            src.Text = num2str(app.current_camera2lidar_6dof(2));
        end
    end % onRotYChangedCallback

    function onRotXChangedCallback(src, data)
        value = str2double(src.Text);
        if value < 3.14 & value > -3.14
            app.current_camera2lidar_6dof(3) = value;
            app.updateImageView();
        else
            src.Text = num2str(app.current_camera2lidar_6dof(3));
        end
    end % onRotXChangedCallback

    function onVecXChangedCallback(src, data)
        value = str2double(src.Text);
        if value < 10 & value > -10
            app.current_camera2lidar_6dof(4) = value;
            app.updateImageView();
        else
            src.Text = num2str(app.current_camera2lidar_6dof(4));
        end
    end % onVecXChangedCallback

    function onVecYChangedCallback(src, data)
        value = str2double(src.Text);
        if value < 10 & value > -10
            app.current_camera2lidar_6dof(5) = value;
            app.updateImageView();
        else
            src.Text = num2str(app.current_camera2lidar_6dof(5));
        end
    end % onVecYChangedCallback

    function onVecZChangedCallback(src, data)
        value = str2double(src.Text);
        if value < 10 & value > -10
            app.current_camera2lidar_6dof(6) = value;
            app.updateImageView();
        else
            src.Text = num2str(app.current_camera2lidar_6dof(6));
        end
    end % onVecZChangedCallback

    function onAddRotZ(src, data, app, RotZEditField)
        if app.current_camera2lidar_6dof(1) + 0.01 < 3.14
            app.current_camera2lidar_6dof(1)  = app.current_camera2lidar_6dof(1) + 0.01;
            RotZEditField.Value = num2str(app.current_camera2lidar_6dof(1));
            app.updateImageView();
        end
    end % onAddRotZ

    function onSubRotZ(src, data, app, RotZEditField)
        if app.current_camera2lidar_6dof(1) - 0.01 > -3.14
            app.current_camera2lidar_6dof(1)  = app.current_camera2lidar_6dof(1) - 0.01;
            RotZEditField.Value = num2str(app.current_camera2lidar_6dof(1));
            app.updateImageView();
        end
    end % onSubRotZ

    function onAddRotY(src, data, app, RotYEditField)
        if app.current_camera2lidar_6dof(2) + 0.01 < 3.14
            app.current_camera2lidar_6dof(2)  = app.current_camera2lidar_6dof(2) + 0.01;
            RotYEditField.Value = num2str(app.current_camera2lidar_6dof(2));
            app.updateImageView();
        end
    end % onAddRotY

    function onSubRotY(src, data, app, RotYEditField)
        if app.current_camera2lidar_6dof(2) - 0.01 > -3.14
            app.current_camera2lidar_6dof(2)  = app.current_camera2lidar_6dof(2) - 0.01;
            RotYEditField.Value = num2str(app.current_camera2lidar_6dof(2));
            app.updateImageView();
        end
    end % onSubRotY

    function onAddRotX(src, data, app, RotXEditField)
        if app.current_camera2lidar_6dof(3) + 0.01 < 3.14
            app.current_camera2lidar_6dof(3)  = app.current_camera2lidar_6dof(3) + 0.01;
            RotXEditField.Value = num2str(app.current_camera2lidar_6dof(3));
            app.updateImageView();
        end
    end % onAddRotX

    function onSubRotX(src, data, app, RotXEditField)
        if app.current_camera2lidar_6dof(3) - 0.01 > -3.14
            app.current_camera2lidar_6dof(3)  = app.current_camera2lidar_6dof(3) - 0.01;
            RotXEditField.Value = num2str(app.current_camera2lidar_6dof(3));
            app.updateImageView();
        end
    end % onSubRotX

    function onAddVecX(src, data, app, VecXEditField)
        if app.current_camera2lidar_6dof(4) + 0.01 < 10
            app.current_camera2lidar_6dof(4)  = app.current_camera2lidar_6dof(4) + 0.01;
            VecXEditField.Value = num2str(app.current_camera2lidar_6dof(4));
            app.updateImageView();
        end
    end % onAddVecX

    function onSubVecX(src, data, app, VecXEditField)
        if app.current_camera2lidar_6dof(4) - 0.01 > -10
            app.current_camera2lidar_6dof(4)  = app.current_camera2lidar_6dof(4) - 0.01;
            VecXEditField.Value = num2str(app.current_camera2lidar_6dof(4));
            app.updateImageView();
        end
    end % onSubVecX

    function onAddVecY(src, data, app, VecYEditField)
        if app.current_camera2lidar_6dof(5) + 0.01 < 10
            app.current_camera2lidar_6dof(5)  = app.current_camera2lidar_6dof(5) + 0.01;
            VecYEditField.Value = num2str(app.current_camera2lidar_6dof(5));
            app.updateImageView();
        end
    end % onAddVecY

    function onSubVecY(src, data, app, VecYEditField)
        if app.current_camera2lidar_6dof(5) - 0.01 > -10
            app.current_camera2lidar_6dof(5)  = app.current_camera2lidar_6dof(5) - 0.01;
            VecYEditField.Value = num2str(app.current_camera2lidar_6dof(5));
            app.updateImageView();
        end
    end % onSubVecY

    function onAddVecZ(src, data, app, VecZEditField)
        if app.current_camera2lidar_6dof(6) + 0.01 < 10
            app.current_camera2lidar_6dof(6)  = app.current_camera2lidar_6dof(6) + 0.01;
            VecZEditField.Value = num2str(app.current_camera2lidar_6dof(6));
            app.updateImageView();
        end
    end % onAddVecZ

    function onSubVecZ(src, data, app, VecZEditField)
        if app.current_camera2lidar_6dof(6) - 0.01 > -10
            app.current_camera2lidar_6dof(6)  = app.current_camera2lidar_6dof(6) - 0.01;
            VecZEditField.Value = num2str(app.current_camera2lidar_6dof(6));
            app.updateImageView();
        end
    end % onSubVecZ

    function onRefreshExtrinsic(x, y, app, RotZEditField, ...
                                            RotYEditField, ...
                                            RotXEditField, ...
                                            VecXEditField, ...
                                            VecYEditField, ...
                                            VecZEditField);
        if app.current_camera2lidar_6dof(1) > -3.14 & app.current_camera2lidar_6dof(1) < 3.14
            RotZEditField.Value = num2str(app.current_camera2lidar_6dof(1));
        end

        if app.current_camera2lidar_6dof(2) > -3.14 & app.current_camera2lidar_6dof(2) < 3.14
            RotYEditField.Value = num2str(app.current_camera2lidar_6dof(2));
        end

        if app.current_camera2lidar_6dof(3) > -3.14 & app.current_camera2lidar_6dof(3) < 3.14
            RotXEditField.Value = num2str(app.current_camera2lidar_6dof(3));
        end

        if app.current_camera2lidar_6dof(4) > -10 & app.current_camera2lidar_6dof(4) < 10
            VecXEditField.Value = num2str(app.current_camera2lidar_6dof(4));
        end

        if app.current_camera2lidar_6dof(5) > -10 & app.current_camera2lidar_6dof(5) < 10
            VecYEditField.Value = num2str(app.current_camera2lidar_6dof(5));
        end

        if app.current_camera2lidar_6dof(6) > -10 & app.current_camera2lidar_6dof(6) < 10
            VecZEditField.Value = num2str(app.current_camera2lidar_6dof(6));
        end

        app.updateImageView();
    end

    function onReprojectionTypeChangedCallback(src, data)
        value = src.SelectedItem;
        if isequal(value, 'U')
            app.current_reprojection_type = 'undistort';
        elseif isequal(value, 'D')
            app.current_reprojection_type = 'distort';
        end
        app.updateImageView();
    end % onReprojectionTypeChangedCallback
end

%-------------------------------------------------------------------------%
function setDefaultLayout(src, data, app)
    % reset the layout to default
    
    % app.setDefaultLayout();
end  % setDefaultLayout