function tabgroup = chessboard_camera_calibration_buildTabGroup(app)
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
    createCalibration(tabCalibration, app)

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
    % add file add push button
    % built-in: see matlab.ui.internal.toolstrip.Icon.showStandardIcons()
    ImagesAddButton = SplitButton(sprintf('Add\nImages'), Icon(javaObjectEDT('javax.swing.ImageIcon', fullfile(app.current_dir, 'icons', 'add_chessboard_30.png'))));
    ImagesAddButton.Tag = 'ImagesAddButton';
    ImagesAddButton.Description = 'add images';
    ImagesAddButton.DynamicPopupFcn = @(x,y) buildImagesAddButtonDynamicPopupList_SmallIcon(app, 'swing');
    % add file close push button
    % built-in: see Icon.showStandardIcons()
    ImagesClearButton = Button(sprintf('Clear\nImages'), Icon(javaObjectEDT('javax.swing.ImageIcon', fullfile(app.current_dir, 'icons', 'clear_chessboard_30.png'))));
    ImagesClearButton.Tag = 'ImagesClearButton';
    ImagesClearButton.Description = 'clear the Images';

    % add load intrinsic file button
    % built-in: see Icon.showStandardIcons()
    LoadIntrinsicButton = Button(sprintf('Load\nIntrinsic'), Icon.ADD_24);
    LoadIntrinsicButton.Tag = 'LoadIntrinsic';
    LoadIntrinsicButton.Description = 'Load Intrinsic yml';

    % assemble
    add(tab, section);
    add(section, column1);
    add(column1, ImagesAddButton);
    add(section, column2);
    add(column2, ImagesClearButton);
    add(section, column3);
    add(column3, LoadIntrinsicButton);

    % add callback
    ImagesClearButton.ButtonPushedFcn = @(x, y) clearImages(x, y, app);
    LoadIntrinsicButton.ButtonPushedFcn = @(x, y) onLoadIntrinsic(x, y, app);

    %-------------------------------------------------------------------------%
    function clearImages(src, data, app)
        % init the image render type
        app.current_image_show_type = 'raw';

        % init data
        app.initData();

        % empty the figure and table
        app.initView();
    end  % clearPair

    %-------------------------------------------------------------------------%
    function onLoadIntrinsic(src, data, app)
        % choose a folder to save the configuration file
        [FileName, PathName] = uigetfile('*.yml', 'load intrinsic parameters file', [app.current_dir]);
        if isequal(FileName, 0)
            disp(myLog('user selected cancel'));
            return;
        else
            disp(myLog(['user selected ', [PathName FileName]]));
        end
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

function popup = buildImagesAddButtonDynamicPopupList_SmallIcon(app, mode)
    import matlab.ui.internal.toolstrip.*
    popup = PopupList();
    item = ListItem('add from file', Icon(javaObjectEDT('javax.swing.ImageIcon', fullfile(app.current_dir, 'icons', 'add_chessboard_16.png'))));
    item.Description = 'add from file';
    item.ShowDescription = false;
    item.ItemPushedFcn = @(x, y) addImagesFromFile(x, y, app);
    popup.add(item);

    %-------------------------------------------------------------------------%
    function addImagesFromFile(src, data, app)
        % create a dialog that get the images\
        d = dialog('Position',[700 400 500 200],'Name','Load Images');

        chessboardSquareMetric = 'millimeters';
        chessboardSquareValue = app.chessboardExtractionParams.chessboardProperties(3) * 1000;
        chessboardSquareNumX = app.chessboardExtractionParams.chessboardProperties(1);
        chessboardSquareNumY = app.chessboardExtractionParams.chessboardProperties(2);

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
                app.chessboardExtractionParams.chessboardProperties(3) = chessboardSquareValue / 1000;
            case 'centimeters'
                app.chessboardExtractionParams.chessboardProperties(3) = chessboardSquareValue / 100;
            otherwise % default the metric to millimeters
                app.chessboardExtractionParams.chessboardProperties(3) = chessboardSquareValue / 1000;
            end

            % set the pattern size of the chessboard
            app.chessboardExtractionParams.chessboardProperties(1) = chessboardSquareNumX;
            app.chessboardExtractionParams.chessboardProperties(2) = chessboardSquareNumY;

            % delete the figure
            delete(gcf);

            % init the image render type
            app.current_image_show_type = 'raw';

            % init data
            app.initData();

            % empty the figure and table
            app.initView();

            % load the image-point pairs
            imageStruct = loadImages(app.current_images_dir);
            if isempty(imageStruct)
                return
            end

            app.imageStruct = imageStruct;

            app.current_imageFilenames = imageStruct.rawImagesFilename;
            app.current_enables = cell(length(app.current_imageFilenames));
            for i = 1 : length(app.current_enables)
                app.current_enables{i} = false;
            end

            % update the current_imageSize
            app.current_imageSize = size(imread(app.current_imageFilenames{1}));

            % refresh the content of the browser table
            app.refreshBrowserTableData();

            app.current_index = 1;

            app.updateImage();
            % refresh the camera image chessboard points
            app.updateImageView();

            % refresh the image points table view
            app.refreshImagePointsTableData();
        end % onOK

    end  % addImagesFromFile
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

    checkboxAllSelectedImages = CheckBox('AllSelected');
    checkboxAllSelectedImages.Description = 'AllSelected';
    checkboxAllSelectedImages.ValueChangedFcn = @checkboxAllSelectedChangedCallback;

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
    add(column3, checkboxAllSelectedImages);

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

            if isequal(app.current_image_show_type, 'raw')
                buttonImageRaw.Value = true;
            elseif isequal(app.current_image_show_type, 'extracted')
                buttonImageExtracted.Value = true;
            elseif isequal(app.current_image_show_type, 'undistorted')
                buttonImageUndistorted.Value = true;
            else
                buttonImageRaw.Value = true;
            end
        end
    end % imageTypeRawPropertyChangedCallback

    function imageTypeExtractedPropertyChangedCallback(src, data)
        if data.EventData.NewValue
            if ~isequal(app.current_image_show_type, 'extracted')
                if ~isempty(app.cameraIntrinsicCalibrationData.imagePoints)
                    app.current_image_show_type = 'extracted';
                    app.updateImageView();
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
            else
                buttonImageRaw.Value = true;
            end
        end

        function warningNoneIntrinsicCalibrationDialog
            str = sprintf('None Intrinsic Calinration Data Found! \r\nDo intrinsic calibration before viewing the undistorted data');
            uiwait(warndlg(str, 'None Intrinsic Calibration'));
        end % warningNoneIntrinsicCalibrationDialog
    end % imageTypeUndistortedPropertyChangedCallback

    function checkboxAllSelectedChangedCallback(src, data)
        if(isempty(app.current_imageFilenames))
            warningNoImagesDialog();
            if src.Selected
                src.Selected = false;
            else
                src.Selected = true;
            end
            return;
        end
        if isequal(data.EventData.NewValue, 1)
            % enable all the images
            for i = 1 : length(app.current_enables)
                app.current_enables{i} = true;
            end
        else
            % cancelled all the images
            for i = 1 : length(app.current_enables)
                app.current_enables{i} = false;
            end
        end
        % update the browser
        % refresh the content of the browser table
        app.refreshBrowserTableData();

        function warningNoImagesDialog
            str = sprintf('No Images!');
            uiwait(warndlg(str, ' Please first load images'));
        end % warningNoImagesDialog
    end % checkboxAllSelectedChangedCallback

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
    CalibrateIntrinsicButton = Button(sprintf('Calibrate\nIntrinsic'), Icon.RUN_24);
    CalibrateIntrinsicButton.Tag = 'calibrateIntrinsic';
    CalibrateIntrinsicButton.Description = 'calibrateIntrinsic';
    CalibrateIntrinsicButton.ButtonPushedFcn = @onCalibrateIntrinsic;

    % built-in: see matlab.ui.internal.toolstrip.Icon.showStandardIcons()
    exportCalibrationParametersButton = SplitButton(sprintf('Export Calibration\nParameters'), Icon.CONFIRM_24);
    exportCalibrationParametersButton.Tag = 'exportCalibrationParametersButton';
    exportCalibrationParametersButton.Description = 'export calibration parameters';
    exportCalibrationParametersButton.DynamicPopupFcn = @(x,y) buildDynamicexportCalibrationParametersPopupList_SmallIcon(app, 'swing');

    % built-in: see matlab.ui.internal.toolstrip.Icon.showStandardIcons()
    exportUndistortedImagesButton = Button(sprintf('Export\nUndistorted'), Icon.RUN_24);
    exportUndistortedImagesButton.Tag = 'exportUndistorted';
    exportUndistortedImagesButton.Description = 'exportUndistorted';
    exportUndistortedImagesButton.ButtonPushedFcn = @onExportUndistorted;

    % assemble
    add(tab, section);
    add(section, column1);
    add(column1, ExtractButton);
    add(section, column2);
    add(column2, CalibrateIntrinsicButton);
    add(section, column3);
    add(column3, exportCalibrationParametersButton);
    add(section, column4);
    add(column4, exportUndistortedImagesButton);

    function onExtract(src, data)
        if isempty(app.imageStruct)
            warningNoneImageDataDialog();
            return;
        end
        % detect the chessboard in images
        patternSize = (app.chessboardExtractionParams.chessboardProperties(1) - 1) * ...
                        (app.chessboardExtractionParams.chessboardProperties(2) - 1);
        [imagePointsConsistency, boardSizeConsistency, imageUsedConsistency, imagePoints, boardSize, imageUsed] = detectImageChessboardPoints(  app.imageStruct.rawImagesFilename, ...
                                                                                                                                                'matlab', ...
                                                                                                                                                patternSize);

        % extract the final matching pairs
        matchedIndex = imageUsedConsistency;

        % save the data for intrinsic calibration
        imagePoints_ = imagePoints(matchedIndex);

        app.cameraIntrinsicCalibrationData.imagePoints = imagePoints_;

        app.cameraIntrinsicCalibrationData.worldPoints = generateCheckerboardPoints(boardSizeConsistency, app.chessboardExtractionParams.chessboardProperties(3)); 
        imageFileNames = app.imageStruct.rawImagesFilename(matchedIndex);
        if isempty(imageFileNames)
            warningNoneImageDataDialog();
            return;
        end
        originalImage = imread(imageFileNames{1});
        [mrows, ncols, ~] = size(originalImage);
        app.cameraIntrinsicCalibrationData.ImageSize = [mrows ncols];

        app.cameraIntrinsicCalibrationData.enable = cell(length(app.cameraIntrinsicCalibrationData.imagePoints), 1);
        for i = 1 : length(app.cameraIntrinsicCalibrationData.enable)
            app.cameraIntrinsicCalibrationData.enable{i} = true;
        end

        app.current_imageFilenames = imageFileNames;
        app.current_enables = app.cameraIntrinsicCalibrationData.enable;

        % update the current_imageSize
        app.current_imageSize = size(imread(app.current_imageFilenames{1}));

        % refresh the content of the browser table
        app.refreshBrowserTableData();

        app.current_index = 1;

        app.updateImage();
        % refresh the camera image chessboard points
        app.updateImageView();

        % refresh the image points table view
        app.refreshImagePointsTableData();

        function warningNoneImageDataDialog
            str = sprintf('None Image Data Found! \r\nLoad the image data first');
            uiwait(warndlg(str, 'None Image Data'));
        end % warningNoneImageDataDialog
    end % onExtract

    function onCalibrateIntrinsic(src, data)
        if(isempty(app.cameraIntrinsicCalibrationData.imagePoints))
            warningNoExtractionDataDialog();
            return;
        end
        % start to calibrate, set the cursor to busy status
        set(app.guiBrowserTable.handle, 'Pointer', 'watch');
        set(app.guiImageView.handle, 'Pointer', 'watch');
        set(app.guiImageDataView.handle, 'Pointer', 'watch');
        drawnow;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % do intrinsic calibration
        % re-select the data according to the enables
        imagePoints_ = [];
        for i = 1 : length(app.cameraIntrinsicCalibrationData.enable)
            if app.cameraIntrinsicCalibrationData.enable{i}
                imagePoints_ = cat(3, imagePoints_, app.cameraIntrinsicCalibrationData.imagePoints{i});
            end
        end
        % calibrate the camera
        [cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(...
            imagePoints_, ...
            app.cameraIntrinsicCalibrationData.worldPoints, ...
            'EstimateSkew', app.cameraIntrinsicCalibrationData.EstimateSkew, ...
            'EstimateTangentialDistortion', app.cameraIntrinsicCalibrationData.EsitmateTangentialDistortion, ...
            'NumRadialDistortionCoefficients', app.cameraIntrinsicCalibrationData.NumRadialDistortionCoefficients, ...
            'WorldUnits', app.cameraIntrinsicCalibrationData.WorldUnits, ...
            'InitialIntrinsicMatrix', app.cameraIntrinsicCalibrationData.InitialIntrinsicMatrix, ...
            'InitialRadialDistortion', app.cameraIntrinsicCalibrationData.InitialRadialDistortion, ...
            'imageSize', app.cameraIntrinsicCalibrationData.ImageSize);
        
        app.cameraIntrinsicCalibrationResult.cameraParams = cameraParams;
        app.cameraIntrinsicCalibrationResult.imagesUsed = imagesUsed;
        app.cameraIntrinsicCalibrationResult.estimationErrors = estimationErrors;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % finish calibrating, set the cursor to arrow
        set(app.guiBrowserTable.handle, 'Pointer', 'arrow');
        set(app.guiImageView.handle, 'Pointer', 'arrow');
        set(app.guiImageDataView.handle, 'Pointer', 'arrow');
        drawnow;

        warningCalibrationDoneDialog();

        app.refreshCalibrationResultsTableData();

        function warningCalibrationDoneDialog
            str = sprintf('Calibration Done!');
            uiwait(warndlg(str, ' Calibration Done'));
        end % warningNoneCalibrationDialog

        function warningNoExtractionDataDialog
            str = sprintf('No extraction data!');
            uiwait(warndlg(str, ' Do extraction before calibration'));
        end % warningNoExtractionDataDialog
    end % onCalibrateIntrinsic

    function popup = buildDynamicexportCalibrationParametersPopupList_SmallIcon(app, mode)
        import matlab.ui.internal.toolstrip.*;
        popup = PopupList();

        item2 = ListItem('intrinsic yml', Icon(javaObjectEDT('javax.swing.ImageIcon', fullfile(app.current_dir, 'icons', 'export_autoware_24.png'))));
        item2.Description = 'intrinsic autoware style';
        item2.ShowDescription = false;
        item2.ItemPushedFcn = @(x, y) exportAutowareYamlIntrinsic(x, y, app);
        popup.add(item2);

        function exportAutowareYamlIntrinsic(src, data, app)
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
                % cameraParams.ImageSize: [rows cols]
                imageSize = [   app.cameraIntrinsicCalibrationResult.cameraParams.ImageSize(1) ...
                                app.cameraIntrinsicCalibrationResult.cameraParams.ImageSize(2)];
            else
                warningNoneIntrinsicCalibrationDialog();
                return;
            end

            % choose a folder to save the configuration file
            [FileName, PathName] = uiputfile('test.yml', 'save intrinsic parameters file', [app.current_dir, '/test.yml']);
            if isequal(FileName, 0)
                disp(myLog('user selected cancel'));
                return;
            else
                disp(myLog(['user selected ', [PathName FileName]]));
            end
            exportAutowareStyleYmlIntrinsic();
            

            function warningNoneIntrinsicCalibrationDialog
                str = sprintf('None Intrinsic Calibration Data Found! \r\nDo intrinsic calibration first!');
                uiwait(warndlg(str, 'None Intrinsic Calibration'));
            end % warningNoneIntrinsicCalibrationDialog

            function warningCalibrationFileDoneDialog
                str = sprintf('Export Calibration File Done! \r\n');
                uiwait(warndlg(str, 'Export Calibration File Done'));
            end % warningCalibrationFileDoneDialog

            function exportAutowareStyleYmlIntrinsic
                % export the parameters to the yaml file
                fid = fopen([PathName FileName], 'w');
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
                str = sprintf('ReprojectionError: %.6f\r\n', app.cameraIntrinsicCalibrationResult.cameraParams.MeanReprojectionError);
                fprintf(fid, str);
                fclose(fid);

                warningCalibrationFileDoneDialog();
            end

        end % exportAutowareStyleYmlIntrinsic
    end

    function onExportUndistorted(src, data)
        % check the images and intrinsics
        if(isempty(app.imageStruct.rawImagesFilename))
            warningNoImagesDialog();
            return;
        end

        if(isempty(app.cameraIntrinsicCalibrationResult.cameraParams))
            warningNoIntrinsicDialog();
            return;
        end

        % select the direcotory of the undistorted images
        folder_name = uigetdir(app.current_undistorted_images_dir, 'Select folder to export the undistorted images');
        if isequal(folder_name, 0)
            disp(myLog('user selected cancel'));
            return;
        else
            disp(myLog(['user selected ', folder_name]));
            app.current_undistorted_images_dir = folder_name;
        end

        h = waitbar(0, 'export undistorted images', 'CreateCancelBtn',@waitbar_cancel, 'Name', 'export undistorted images');

        % get the current images and enables
        imagesNum = length(app.imageStruct.rawImagesFilename);
        ifCancel = false;
        for i = 1 : imagesNum
            if ~ifCancel
                current_image_raw_file_full = app.imageStruct.rawImagesFilename{i};
                current_image_raw = imread(current_image_raw_file_full);
                % undistorted the images
                current_undistortedImage = undistortImage(current_image_raw, ...
                                                            app.cameraIntrinsicCalibrationResult.cameraParams);
                % generate the undistorted image filename
                current_image_raw_file_full_splited = split(current_image_raw_file_full, '/');
                current_image_raw_file = current_image_raw_file_full_splited{end};
                current_image_undistorted_file_full = [app.current_undistorted_images_dir '/' current_image_raw_file];

                imwrite(current_undistortedImage, current_image_undistorted_file_full);

                progress = i/imagesNum;
                msgs = sprintf('export undistorted image %d/%d', i, imagesNum);
                waitbar(progress, h, msgs, 'Name', 'export undistorted images');
            end
        end
        delete(h);

        function waitbar_cancel(src, event)
            %global ifCancel;
            ifCancel = true;
            delete(src);
        end

        function warningNoImagesDialog
            str = sprintf('No Images!');
            uiwait(warndlg(str, ' Please first load images'));
        end % warningNoImagesDialog

        function warningNoIntrinsicDialog
            str = sprintf('No Intrinsic!');
            uiwait(warndlg(str, ' Do calibration before exporting undistorted'));
        end % warningNoIntrinsicDialog
    end % onExportUndistorted

end % createExtrinsicCalibration

%-------------------------------------------------------------------------%
function setDefaultLayout(src, data, app)
    % reset the layout to default
    
    % app.setDefaultLayout();
end  % setDefaultLayout