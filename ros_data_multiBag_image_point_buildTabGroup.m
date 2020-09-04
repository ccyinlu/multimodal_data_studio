function tabgroup = ros_data_multiBag_image_point_buildTabGroup(app)
    % Build the TabGroup for the ros_data_multiBag_image_point.

    % Author(s): ethan
    % Copyright ccyinlu@whu.edu.cn.
    % Date: 20190102
    import matlab.ui.internal.toolstrip.*
    % tab group
    tabgroup = TabGroup();
    % tabs
    tabRosbag = Tab('Rosbag');
    tabRosbag.Tag = 'tabRosbag';

    createFile(tabRosbag, app);
    createControl(tabRosbag, app);
    createParameter(tabRosbag, app);
    createView(tabRosbag, app);

    % assemble
    tabgroup.add(tabRosbag);
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
    column5 = Column();
    column6 = Column();
    % add file add push button
    % built-in: see matlab.ui.internal.toolstrip.Icon.showStandardIcons()
    FileAddIcon = Icon.ADD_24;
    FileAddButton = Button('Add', FileAddIcon);
    FileAddButton.Tag = 'FileAddButton';
    FileAddButton.Description = 'add rosbag files';
    % add file DELETE push button
    % built-in: see matlab.ui.internal.toolstrip.Icon.showStandardIcons()
    FileDelIcon = Icon.DELETE_24;
    FileDelButton = Button('Del', FileDelIcon);
    FileDelButton.Tag = 'FileDelButton';
    FileDelButton.Description = 'delete rosbag files';
    % add file close push button
    % built-in: see Icon.showStandardIcons()
    FileCloseIcon = Icon.CLOSE_24;
    FileCloseButton = Button('Close', FileCloseIcon);
    FileCloseButton.Tag = 'FileCloseButton';
    FileCloseButton.Description = 'close the rosbag';
    % add file save push button
    % built-in: see Icon.showStandardIcons()
    FileSaveIcon = Icon.SAVE_24;
    FileSaveButton = Button('Save', FileSaveIcon);
    FileSaveButton.Tag = 'FileSaveButton';
    FileSaveButton.Description = 'save ros topics';

    % add file save single shot button
    % built-in: see Icon.showStandardIcons()
    imageFile = fullfile(app.current_dir, 'icons', 'file_16.png');
    SelectSingleDirButton = Button('SBrowse', Icon(javaObjectEDT('javax.swing.ImageIcon',imageFile)));
    SelectSingleDirButton.Tag = 'SelectDirButton';
    SelectSingleDirButton.Description = 'select single shot dir';

    imageFile = fullfile(app.current_dir, 'icons', 'shot_16.png');
    SingleShotButton = Button('SShot', Icon(javaObjectEDT('javax.swing.ImageIcon',imageFile)));
    SingleShotButton.Tag = 'SingleShotButton';
    SingleShotButton.Description = 'save the current selected topics';

    imageFile = fullfile(app.current_dir, 'icons', 'reset_16.png');
    ResetSingleShotButton = Button('Reset', Icon(javaObjectEDT('javax.swing.ImageIcon',imageFile)));
    ResetSingleShotButton.Tag = 'ResetSingleShotButton';
    ResetSingleShotButton.Description = 'reset the shot index';

    % assemble
    add(tab, section);
    add(section, column1);
    add(column1, FileAddButton);
    add(section, column2);
    add(column2, FileDelButton);
    add(section, column3);
    add(column3, FileCloseButton);
    add(section, column4);
    add(column4, FileSaveButton);
    add(section, column5);
    add(column5, SelectSingleDirButton);
    add(column5, SingleShotButton);
    add(column5, ResetSingleShotButton);

    % add callback
    FileAddButton.ButtonPushedFcn = @(x, y) AddRosbagFiles(x, y, app);
    FileDelButton.ButtonPushedFcn = @(x, y) DelRosbagFiles(x, y, app);
    FileCloseButton.ButtonPushedFcn = @(x, y) closeRosbagFiles(x, y, app);
    FileSaveButton.ButtonPushedFcn = @(x, y) saveRosbagTopics(x, y, app);

    SelectSingleDirButton.ButtonPushedFcn = @(x, y) selectSingleShotDir(x, y, app);
    SingleShotButton.ButtonPushedFcn = @(x, y) SingleShotImage(x, y, app);
    ResetSingleShotButton.ButtonPushedFcn = @(x, y) ResetSingleShot(x, y, app);
end

function createView(tab, app)
    import matlab.ui.internal.toolstrip.*
    % create section
    section = Section('View');
    section.Tag = 'ViewSection';
    % create column
    column1 = Column();
    % add set default view layout button
    ViewDefaultLayoutIcon = Icon.LAYOUT_16;
    ViewDefaultLayoutButton = Button('Default Layout', ViewDefaultLayoutIcon);
    ViewDefaultLayoutButton.Tag = 'ViewDefaultLayoutButton';
    ViewDefaultLayoutButton.Description = 'set the default layout';
    
    % assemble
    add(tab, section);
    add(section, column1);
    add(column1, ViewDefaultLayoutButton);
    % add callback
    ViewDefaultLayoutButton.ButtonPushedFcn = @(x, y) setDefaultLayout(x, y, app);
end

function createParameter(tab, app)
    import matlab.ui.internal.toolstrip.*
    % create section
    section = Section('Parameter');
    section.Tag = 'parameterSection';
    % create column
    column1 = Column('HorizontalAlignment', 'right');
    column2 = Column('Width', 50);
    column3 = Column('Width', 60, 'HorizontalAlignment', 'center');
    column4 = Column('Width', 80, 'HorizontalAlignment', 'center');

    column5 = Column('Width', 60, 'HorizontalAlignment', 'right');
    column6 = Column('Width', 50);

    column7 = Column('Width', 60, 'HorizontalAlignment', 'right');
    column8 = Column('Width', 50);

    column9 = Column('Width', 60, 'HorizontalAlignment', 'right');
    column10 = Column('Width', 50);

    column11 = Column('Width', 60, 'HorizontalAlignment', 'right');
    column12 = Column('Width', 50);

    column13 = Column('Width', 80, 'HorizontalAlignment', 'right');
    column14 = Column('Width', 50);

    labelPlayStep = Label('play step:');
    labelTimeThreshold = Label('time TH:');
    labelPlayPause = Label('play pause:');

    % add play parameters
    playStepEditField = EditField();
    playStepEditField.Value = '0.1';
    playStepEditField.Description = 'set play step';

    timeThEditField = EditField();
    timeThEditField.Value = '0.1';
    timeThEditField.Description = 'time threshold';

    playPauseEditField = EditField();
    playPauseEditField.Value = '0.0';
    playPauseEditField.Description = 'play pause';

    % add point view control parameter
    labelPointSize = Label('point size');

    pointSizeSlider = Slider([1 10],1);
    pointSizeSlider.Steps = 1;

    pointSizeEditField = EditField();
    pointSizeEditField.Value = '1';
    pointSizeEditField.Description = 'point size';

    % add render type
    labelPointRenderType = Label('render type');

    pointRenderType = DropDown({'P' 'plain';'H' 'height';'R' 'range';'I' 'intensity'});
    pointRenderType.Description = 'point render type';
    pointRenderType.Editable = true;
    pointRenderType.Value = 'R';

    labelXLimMin = Label('XLimMin:');
    labelXLimMax = Label('XLimMax:');

    XLimMinEditField = EditField();
    XLimMinEditField.Value = '-100';
    XLimMinEditField.Description = 'XLimMin';

    XLimMaxEditField = EditField();
    XLimMaxEditField.Value = '100';
    XLimMaxEditField.Description = 'XLimMax';

    labelYLimMin = Label('YLimMin:');
    labelYLimMax = Label('YLimMax:');

    YLimMinEditField = EditField();
    YLimMinEditField.Value = '-100';
    YLimMinEditField.Description = 'YLimMin';

    YLimMaxEditField = EditField();
    YLimMaxEditField.Value = '100';
    YLimMaxEditField.Description = 'YLimMax';

    labelZLimMin = Label('ZLimMin:');
    labelZLimMax = Label('ZLimMax:');

    ZLimMinEditField = EditField();
    ZLimMinEditField.Value = '-20';
    ZLimMinEditField.Description = 'ZLimMin';

    ZLimMaxEditField = EditField();
    ZLimMaxEditField.Value = '20';
    ZLimMaxEditField.Description = 'ZLimMax';

    labelIntensityMin = Label('IMin:');
    labelIntensityMax = Label('Imax:');

    IntensityMinEditField = EditField();
    IntensityMinEditField.Value = '0';
    IntensityMinEditField.Description = 'IMin';

    IntensityMaxEditField = EditField();
    IntensityMaxEditField.Value = '255';
    IntensityMaxEditField.Description = 'Imax';

    labelSyncOffset = Label('SyncOffset:');

    syncOffsetEditField = EditField();
    syncOffsetEditField.Value = '0';
    syncOffsetEditField.Description = 'SyncOffset';
    
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
    

    add(column1, labelPlayStep);
    add(column1, labelTimeThreshold);
    add(column1, labelPlayPause);
    add(column2, playStepEditField);
    add(column2, timeThEditField);
    add(column2, playPauseEditField);
    add(column3, labelPointSize);
    add(column3, pointSizeSlider);
    add(column3, pointSizeEditField);
    add(column4, labelPointRenderType);
    add(column4, pointRenderType);

    add(column5, labelXLimMin);
    add(column5, labelXLimMax);
    add(column6, XLimMinEditField);
    add(column6, XLimMaxEditField);

    add(column7, labelYLimMin);
    add(column7, labelYLimMax);
    add(column8, YLimMinEditField);
    add(column8, YLimMaxEditField);

    add(column9, labelZLimMin);
    add(column9, labelZLimMax);
    add(column10, ZLimMinEditField);
    add(column10, ZLimMaxEditField);

    add(column11, labelIntensityMin);
    add(column11, labelIntensityMax);
    add(column12, IntensityMinEditField);
    add(column12, IntensityMaxEditField);

    add(column13, labelSyncOffset);
    add(column14, syncOffsetEditField);
    % add callback
    playStepEditField.ValueChangedFcn = @(x, y) onPlayStepChangedCallback(x, y, app);
    timeThEditField.ValueChangedFcn = @(x, y) onTimeThChangedCallback(x, y, app);
    playPauseEditField.ValueChangedFcn = @(x, y) onPlayPauseChangedCallback(x, y, app);
    pointSizeSlider.ValueChangedFcn = @(x, y) onPointSizeChangedCallback(x, y, app, pointSizeEditField);
    pointSizeEditField.ValueChangedFcn = @(x, y) onPointSizeEditChangedCallback(x, y, app, pointSizeSlider);
    pointRenderType.ValueChangedFcn = @(x, y) onPointRenderTypeChangedCallback(x, y, app);

    XLimMinEditField.ValueChangedFcn =  @(x, y) onXLimMinChangedCallback(x, y, app);
    XLimMaxEditField.ValueChangedFcn =  @(x, y) onXLimMaxChangedCallback(x, y, app);
    YLimMinEditField.ValueChangedFcn =  @(x, y) onYLimMinChangedCallback(x, y, app);
    YLimMaxEditField.ValueChangedFcn =  @(x, y) onYLimMaxChangedCallback(x, y, app);
    ZLimMinEditField.ValueChangedFcn =  @(x, y) onZLimMinChangedCallback(x, y, app);
    ZLimMaxEditField.ValueChangedFcn =  @(x, y) onZLimMaxChangedCallback(x, y, app);

    IntensityMinEditField.ValueChangedFcn =  @(x, y) onIntensityMinChangedCallback(x, y, app);
    IntensityMaxEditField.ValueChangedFcn =  @(x, y) onIntensityMaxChangedCallback(x, y, app);

    syncOffsetEditField.ValueChangedFcn =  @(x, y) onSyncOffsetChangedCallback(x, y, app);

end

function createControl(tab, app)
    import matlab.ui.internal.toolstrip.*
    % create section
    section = Section('Control');
    section.Tag = 'controlSection';
    % create column
    column1 = Column();
    column2 = Column();
    column3 = Column();

    %%%%%%%%%%%%%%%% add timeBase selection radio button
    %% radio button section
    ButtonGroup = ButtonGroup();

    buttonImage = RadioButton(ButtonGroup, 'Image');
    buttonImage.Description = 'image as baseTime';
    % add callback
    buttonImage.ValueChangedFcn = @(x, y) RadioButtonChangedCallback(x, y, app);

    buttonPoint = RadioButton(ButtonGroup, 'Point');
    buttonPoint.Description = 'point as baseTime';
     % add callback
    buttonPoint.ValueChangedFcn = @(x, y) RadioButtonChangedCallback(x, y, app);
    buttonImage.Value = true;

    %%%%%%%%%%%%%%%% add control tool for
    %% button
    % add pointer button
    % built-in: see Icon.showStandardIcons()
    imageFile = fullfile(app.current_dir, 'icons', 'arrow_16.png');
    pointerButton = Button('Pointer', Icon(javaObjectEDT('javax.swing.ImageIcon',imageFile)));
    pointerButton.Tag = 'pointerButton';
    pointerButton.Description = 'pointer';
    % add select button
    % built-in: see Icon.showStandardIcons()
    imageFile = fullfile(app.current_dir, 'icons', 'cross_16.png');
    selectButton = Button('Select', Icon(javaObjectEDT('javax.swing.ImageIcon',imageFile)));
    selectButton.Tag = 'selectButton';
    selectButton.Description = 'select';
    % add data cursor button
    % built-in: see Icon.showStandardIcons()
    imageFile = fullfile(app.current_dir, 'icons', 'cursor_16.png');
    dataCursorButton = Button('Cursor', Icon(javaObjectEDT('javax.swing.ImageIcon',imageFile)));
    dataCursorButton.Tag = 'dataCursorButton';
    dataCursorButton.Description = 'dataCursor';
    % add move button
    % built-in: see Icon.showStandardIcons()
    imageFile = fullfile(app.current_dir, 'icons', 'hand_16.png');
    moveButton = Button('Move', Icon(javaObjectEDT('javax.swing.ImageIcon',imageFile)));
    moveButton.Tag = 'moveButton';
    moveButton.Description = 'move';
     % add zoom button
    % built-in: see Icon.showStandardIcons()
    imageFile = fullfile(app.current_dir, 'icons', 'zoom_out_16.png');
    zoomButton = Button('Zoom', Icon(javaObjectEDT('javax.swing.ImageIcon',imageFile)));
    zoomButton.Tag = 'zoomButton';
    zoomButton.Description = 'zoom';
     % add rotate button
    % built-in: see Icon.showStandardIcons()
    imageFile = fullfile(app.current_dir, 'icons', 'rotate_16.png');
    rotateButton = Button('Rotate', Icon(javaObjectEDT('javax.swing.ImageIcon',imageFile)));
    rotateButton.Tag = 'rotateButton';
    rotateButton.Description = 'rotate';

    % assemble
    add(tab, section);
    add(section, column1);
    add(section, column2);
    add(section, column3);
    add(column1, buttonImage);
    add(column1, buttonPoint);

    add(column2, pointerButton);
    add(column2, selectButton);
    add(column2, dataCursorButton);

    add(column3, moveButton);
    add(column3, zoomButton);
    add(column3, rotateButton);

    pointerButton.ButtonPushedFcn = @(x, y) onPointer(x, y, app);
    selectButton.ButtonPushedFcn = @(x, y) onSelect(x, y, app);
    dataCursorButton.ButtonPushedFcn = @(x, y) onDataCursor(x, y, app);

    moveButton.ButtonPushedFcn = @(x, y) onMove(x, y, app);
    zoomButton.ButtonPushedFcn = @(x, y) onZoom(x, y, app);
    rotateButton.ButtonPushedFcn = @(x, y) onRotate(x, y, app);
end

%-------------------------------------------------------------------------%
function AddRosbagFiles(src, data, app)
    % add the rosbag files
    % open a file selection
    [filename, filepath, fileinx] = uigetfile({'*.bag'}, 'Open Rosbag', app.RosbagCurrentFileName, 'MultiSelect', 'on');
    if isequal(filename, 0)
        disp(myLog('user selected cancel'));
    else
        % start to load the rosbags, set the cursor to busy status
        set(app.guiRosbagTable.handle, 'Pointer', 'watch');
        drawnow;
        % parse the loaded files
        if isa(filename, 'cell')
            fileNum = length(filename);
        else
            fileNum = 1;
        end
        for i = 1 : fileNum
            if fileNum == 1
                disp(myLog(['user selected ', fullfile(filepath, filename)]));
                app.RosbagCurrentFileName = fullfile(filepath, filename);
            else
                disp(myLog(['user selected ', fullfile(filepath, filename{i})]));
                app.RosbagCurrentFileName = fullfile(filepath, filename{i});
            end
            % read the rosbag file
            [bag, index] = loadRosbags(app.RosbagCurrentFileName, app.Rosbag.bags);
            if ~index
                % find a new bag
                app.Rosbag.numBags = app.Rosbag.numBags + 1;
                currentIndex = app.Rosbag.numBags;
                app.Rosbag.bags{currentIndex}               = bag;
                app.Rosbag.filenames{currentIndex}          = app.RosbagCurrentFileName;
                app.Rosbag.availableTopics{currentIndex}    = getTopicsByType(app.Rosbag.bags{currentIndex}, '');
                app.Rosbag.numTopics{currentIndex}          = length(app.Rosbag.availableTopics{currentIndex});
                app.Rosbag.enable{currentIndex}             = false;
                app.Rosbag.startTime{currentIndex}          = app.Rosbag.bags{currentIndex}.StartTime;
                app.Rosbag.endTime{currentIndex}            = app.Rosbag.bags{currentIndex}.EndTime;
                app.Rosbag.duration{currentIndex}           = app.Rosbag.endTime{currentIndex} - app.Rosbag.startTime{currentIndex};
                app.Rosbag.numMessages{currentIndex}        = app.Rosbag.bags{currentIndex}.NumMessages;
            end
        end
        % finished loading the rosbags
        set(app.guiRosbagTable.handle, 'Pointer', 'arrow');
        drawnow;

        % update the rosbag table
        tableData = app.generateRosbagTableData();
        set(app.guiRosbagTable.mTableTopics, 'Data', tableData);

        % update the status table image
        app.updateAvailableImageTopics();
        app.updateAvailableImageTopicsView(1);
        app.updateCurrentImageTopic(1);

        % update the status table point
        app.updateAvailablePointTopics();
        app.updateAvailablePointTopicsView(1);
        app.updateCurrentPointTopic(1);

        % update the play figure info
        app.updatePlayInfo();
        % update the play figure view
        app.updatePlayView();

        % update image and point
        app.updateImagePoint();

        % update the to-be saved topics table of status table
        tableData = app.generateSavedTopicsTableData();
        set(app.guiStatusTable.mTableSavedTopics, 'Data', tableData);
    end
end  % AddRosbagFiles

%-------------------------------------------------------------------------%
function DelRosbagFiles(src, data, app)
    % delete the rosbags
    % re-format the rosbag 
    temp_rosbag = app.Rosbag;
    % clear rosbag
    app.Rosbag.numBags = 0;
    app.Rosbag.bags = {};
    app.Rosbag.filenames = {};
    app.Rosbag.availableTopics = {};
    app.Rosbag.numTopics = {};
    app.Rosbag.enable = {};
    app.Rosbag.startTime = {};
    app.Rosbag.endTime = {};
    app.Rosbag.duration = {};
    app.Rosbag.numMessages = {};

    for index = 1 : temp_rosbag.numBags
        if ~temp_rosbag.enable{index}
            app.Rosbag.numBags = app.Rosbag.numBags + 1;
            currentIndex = app.Rosbag.numBags;
            app.Rosbag.bags{currentIndex}               = temp_rosbag.bags{index};
            app.Rosbag.filenames{currentIndex}          = temp_rosbag.filenames{index};
            app.Rosbag.availableTopics{currentIndex}    = temp_rosbag.availableTopics{index};
            app.Rosbag.numTopics{currentIndex}          = temp_rosbag.numTopics{index};
            app.Rosbag.enable{currentIndex}             = temp_rosbag.enable{index};
            app.Rosbag.startTime{currentIndex}          = temp_rosbag.startTime{index};
            app.Rosbag.endTime{currentIndex}            = temp_rosbag.endTime{index};
            app.Rosbag.duration{currentIndex}           = temp_rosbag.duration{index};
            app.Rosbag.numMessages{currentIndex}        = temp_rosbag.numMessages{index};
        end
    end
    % update the rosbag table
    tableData = app.generateRosbagTableData();
    set(app.guiRosbagTable.mTableTopics, 'Data', tableData);

    % update the status table image
    app.updateAvailableImageTopics();
    app.updateAvailableImageTopicsView(1);
    app.updateCurrentImageTopic(1);

    % update the status table point
    app.updateAvailablePointTopics();
    app.updateAvailablePointTopicsView(1);
    app.updateCurrentPointTopic(1);

    % update the play figure info
    app.updatePlayInfo();
    % update the play figure view
    app.updatePlayView();

    % update image and point
    app.updateImagePoint();

    % update the to-be saved topics table of status table
    tableData = app.generateSavedTopicsTableData();
    set(app.guiStatusTable.mTableSavedTopics, 'Data', tableData);
end % DelRosbagFiles

%-------------------------------------------------------------------------%
function closeRosbagFiles(src, data, app)
    % close the rosbag files
    % init the whole info
    app.initInfo();
    % update the rosbag table
    tableData = app.generateRosbagTableData();
    set(app.guiRosbagTable.mTableTopics, 'Data', tableData);

    % update the status table image
    app.updateAvailableImageTopics();
    app.updateAvailableImageTopicsView(1);
    app.updateCurrentImageTopic(1);

    % update the status table point
    app.updateAvailablePointTopics();
    app.updateAvailablePointTopicsView(1);
    app.updateCurrentPointTopic(1);

    % update the play figure info
    app.updatePlayInfo();
    % update the play figure view
    app.updatePlayView();

    % update image and point
    app.updateImagePoint();

    % update the to-be saved topics table of status table
    tableData = app.generateSavedTopicsTableData();
    set(app.guiStatusTable.mTableSavedTopics, 'Data', tableData);

end  % closeRosbagFiles

%-------------------------------------------------------------------------%
function setDefaultLayout(src, data, app)
    % reset the layout to default
    
    % app.setDefaultLayout();
end  % setDefaultLayout

%-------------------------------------------------------------------------%
function RadioButtonChangedCallback(src, data, app)
    app.RosbagCurrentTimeBaseType = src.Text;
    % update the play figure info
    app.updatePlayInfo();
    % update the play figure view
    app.updatePlayView();
    % update image and point
    app.updateImagePoint();
end  % RadioButtonChangedCallback

function onPointer(src, data, app)
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
        app.dataCursorPointHandle.Enable = 'off';
    end
    drawnow;
end  % onPointer

function onDataCursor(src, data, app)
    ax = gca;
    if isequal(ax, app.guiImageView.axesImageView)
        zoom off;
        rotate3d off;
        app.dataCursorImageHandle = datacursormode(app.guiImageView.handle);
        app.dataCursorImageHandle.UpdateFcn = @(x, y) onCursorImageUpdateFcn(x, y, app);
        app.dataCursorImageHandle.Enable = 'on';
    elseif isequal(ax, app.guiPointView.axesPointView)
        zoom off;
        rotate3d off;
        app.dataCursorPointHandle = datacursormode(app.guiPointView.handle);
        app.dataCursorPointHandle.UpdateFcn = @(x, y) onCursorPointUpdateFcn(x, y, app);
        app.dataCursorPointHandle.DisplayStyle = 'window';
        app.dataCursorPointHandle.Enable = 'on';
    end
    drawnow;
end  % onDataCursor

function onSelect(src, data, app)
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
        app.dataCursorPointHandle.Enable = 'off';
    end
    drawnow;
end  % onSelect

%-------------------------------------------------------------------------%
function onMove(src, data, app)
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
        app.dataCursorPointHandle.Enable = 'off';
    end
    drawnow;
end  % onMove

%-------------------------------------------------------------------------%
function onZoom(src, data, app)
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
        app.dataCursorPointHandle.Enable = 'off';

        % enable zoom 
        h = zoom;
        h.ActionPostCallback = @onZoomPostCallback;
        h.Enable = 'on';
    end
    drawnow;
    function onZoomPostCallback(obj,evd)
       
    end % onZoomPostCallback
end  % onZoom

%-------------------------------------------------------------------------%
function onRotate(src, data, app)
    ax = gca;
    if isequal(ax, app.guiImageView.axesImageView)
        zoom off;
        set(app.dataCursorImageHandle, 'Enable', 'off');
        % delete the datatip
        delete(findall(gcf, 'Type', 'hggroup'));
        rotate3d on;
    elseif isequal(ax, app.guiPointView.axesPointView)
        zoom off;
        app.dataCursorPointHandle.Enable = 'off';
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

function onPlayStepChangedCallback(src, data, app)
    value = str2double(src.Text);
    if value > 0 
        app.RosbagPlayStep = value;
    else
        src.Text = num2str(app.RosbagPlayStep);
    end
end % onPlayStepChangedCallback

function onTimeThChangedCallback(src, data, app)
    value = str2double(src.Text);
    if value > 0 & value < 1
        app.RosbagTimeThreshold = str2double(src.Text);
    else
        src.Text = num2str(app.RosbagTimeThreshold);
    end
end % onTimeThChangedCallback

function onPlayPauseChangedCallback(src, data, app)
    value = str2double(src.Text);
    if value > 0
        app.RosbagPlayInterval = str2double(src.Text);
    else
        src.Text = num2str(app.RosbagPlayInterval);
    end
end % onTimeThChangedCallback

function onPointSizeChangedCallback(src, data, app, pointSizeEditField)
    app.current_point_size = src.Value;
    pointSizeEditField.Value = num2str(src.Value);
    app.updatePointView();
end % onPointSizeChangedCallback

function onPointSizeEditChangedCallback(src, data, app, pointSizeSlider)
    value = str2double(src.Text);
    if value >= 1 & value <= 10
        app.current_point_size = floor(value);
        pointSizeSlider.Value = floor(value);
        app.updatePointView();
    else
        src.Text = num2str(app.current_point_size);
    end
end % onPointSizeChangedCallback

function onPointRenderTypeChangedCallback(src, data, app)
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

function onXLimMinChangedCallback(src, data, app)
    value = str2double(src.Text);
    if value > -200 & value < app.current_point_lim(2)
        app.current_point_lim(1) = value;
        app.updatePointView();
    else
        src.Text = num2str(app.current_point_lim(1));
    end
end % onXLimMinChangedCallback

function onXLimMaxChangedCallback(src, data, app)
    value = str2double(src.Text);
    if value < 200 & value > app.current_point_lim(1)
        app.current_point_lim(2) = value;
        app.updatePointView();
    else
        src.Text = num2str(app.current_point_lim(2));
    end
end % onXLimMaxChangedCallback

function onYLimMinChangedCallback(src, data, app)
    value = str2double(src.Text);
    if value > -200 & value < app.current_point_lim(4)
        app.current_point_lim(3) = value;
        app.updatePointView();
    else
        src.Text = num2str(app.current_point_lim(3));
    end
end % onYLimMinChangedCallback

function onYLimMaxChangedCallback(src, data, app)
    value = str2double(src.Text);
    if value < 200 & value > app.current_point_lim(3)
        app.current_point_lim(4) = value;
        app.updatePointView();
    else
        src.Text = num2str(app.current_point_lim(4));
    end
end % onYLimMaxChangedCallback

function onZLimMinChangedCallback(src, data, app)
    value = str2double(src.Text);
    if value > -200 & value < app.current_point_lim(6)
        app.current_point_lim(5) = value;
        app.updatePointView();
    else
        src.Text = num2str(app.current_point_lim(5));
    end
end % onZLimMinChangedCallback

function onZLimMaxChangedCallback(src, data, app)
    value = str2double(src.Text);
    if value < 200 & value > app.current_point_lim(5)
        app.current_point_lim(6) = value;
        app.updatePointView();
    else
        src.Text = num2str(app.current_point_lim(6));
    end
end % onZLimMaxChangedCallback

function onIntensityMinChangedCallback(src, data, app)
    value = str2double(src.Text);
    if value >= 0 & value < app.current_point_Ilim(2)
        app.current_point_Ilim(1) = value;
        app.updatePointView();
    else
        src.Text = num2str(app.current_point_Ilim(1));
    end
end % onIntensityMinChangedCallback

function onIntensityMaxChangedCallback(src, data, app)
    value = str2double(src.Text);
    if value <= 255 & value > app.current_point_Ilim(1)
        app.current_point_Ilim(2) = value;
        app.updatePointView();
    else
        src.Text = num2str(app.current_point_Ilim(2));
    end
end % onIntensityMaxChangedCallback

function onSyncOffsetChangedCallback(src, data, app)
    value = str2double(src.Text);
    if value >= -10 & value <= 10
        app.current_sync_offset = value;
    else
        src.Text = num2str(app.current_sync_offset);
    end
end % onSyncOffsetChangedCallback

%-------------------------------------------------------------------------%
function saveRosbagTopics(src, data, app)
    % add your code here
    % get the topics to be save
    table = app.guiStatusTable.mTableSavedTopics.Data;
    if size(table, 1)
        % statistic the selected topics
        numTopicsSelected = 0;
        topicsSelected = {};
        bagidSelected = {};
        for i = 1 : size(table, 1)
            if isequal(table{i, 3}, true)
                numTopicsSelected = numTopicsSelected + 1;
                topicsSelected{numTopicsSelected} = table{i, 1};
                bagidSelected{numTopicsSelected} = table{i, 2};
            end
        end
        % validate the topics selected
        % if numTopicsSelected ~= app.current_topics_selected_num
        %     warningTopicsSelectedConfigChangeDialog();
        %     return;
        % else
        %     for i = 1 : numTopicsSelected
        %         if ~isequal(topicsSelected{i}, app.current_topics_selected{i})
        %             warningTopicsSelectedConfigChangeDialog();
        %             return;
        %         end
        %     end
        % end
        if numTopicsSelected
            % select the baseline of the multiple topics
            if isempty(app.RosbagCurrentBaseLineTopic)
                warningNoBaselineSelectedDialog();
                return;
            end
            % check if the baseline topic is selected
            if ~getBelongtoByTopic(app.RosbagCurrentBaseLineTopic, topicsSelected, bagidSelected)
                warningBaselineTopicNotSelectedDialog();
                return;
            end
            % get the dir to save the images
            folder_name = uigetdir(app.RosbagFileSaveDir, 'Select folder to save the topics');
            if isequal(folder_name, 0)
                disp(myLog('user selected cancel'));
                return;
            else
                disp(myLog(['user selected ', folder_name]));
                app.RosbagFileSaveDir = folder_name;
            end
            % create the subfolders
            topic_subfolder = {};
            for i = 1 : numTopicsSelected
                % parse the topic name
                strParse = strsplit(topicsSelected{i}, '/');
                if length(strParse) <= 1
                    warningInvalidTopicNameDialog();
                    return;
                else
                    foldName = strParse{2};
                    topic_subfolder{i} = sprintf('%s/%s', app.RosbagFileSaveDir, foldName);
                    mkdir(topic_subfolder{i});
                end  
            end
            %%%%%%%%%%% start to save the topics %%%%%%%%%%%%%%%%%%%%%%%%
            % get the start time and end time for the baseline_topic
            baselineBag = app.Rosbag.bags{getBelongtoByTopic(app.RosbagCurrentBaseLineTopic, topicsSelected, bagidSelected)};
            baselineBag = select(baselineBag, 'Topic', app.RosbagCurrentBaseLineTopic);
            rosbagStartTime = baselineBag.StartTime;
            startTime = app.RosbagPlaySliderStartTime + rosbagStartTime;
            endTime = app.RosbagPlaySliderEndTime + rosbagStartTime;
            numImageWrite = 0;
            timeStampFile = sprintf('%s/timestamp.txt', app.RosbagFileSaveDir);
            fid = fopen(timeStampFile,'w');
            %global ifCancel;
            ifCancel = false;
            h = waitbar(0, 'Please wait ...', 'CreateCancelBtn',@waitbar_cancel, 'Name', 'Saving the topics');
            % write the header to the timastamp
            str = sprintf('%s\t', 'ID');
            fprintf(fid, str);
            for i = 1 : numTopicsSelected
                str = sprintf('%s\t', topicsSelected{i});
                fprintf(fid, str);
            end
            fprintf(fid, '\r\n');
            currentTime = startTime;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % query the baseline_timestamps by the current_baseline_topic
            %% get the timestamp list for the baseline
            if isequal(app.RosbagCurrentTimeBaseType, 'Image')
                current_baseline_topic = app.RosbagCurrentImageTopic.topic;
            elseif isequal(app.RosbagCurrentTimeBaseType, 'Point')
                current_baseline_topic = app.RosbagCurrentPointTopic.topic;
            end
            for i = 1 : app.current_topics_selected_num
                if isequal(current_baseline_topic, app.current_topics_selected{i})
                    baseline_timestamps = app.current_topics_timestamps{i};
                    baseline_arrived_timestamps = app.current_topics_arrived_timestamps{i};
                    baseline_index = i;
                    break;
                end
            end
            
            while currentTime < endTime
                if ~ifCancel
                    if isequal(app.query_timestamp_type, 'query_by_messageIn_stamp')
                        [~, I] = min(abs(baseline_timestamps(:) - currentTime));
                        current_timestamp_adjust = baseline_timestamps(I); % adjust the current time
                        % first extract the timestamp
                        extractTimestampsFromTopics(app);
                        % first check the all the timestamp compared to the baseline within the threshold, or quit this observation
                        valid_check = true;
                        for k = 1 : numTopicsSelected
                            now_timestamps = app.current_topics_timestamps{k};
                            [~, I] = min(abs(now_timestamps(:) - current_timestamp_adjust));
                            now_timestamps_selected = now_timestamps(I);
                            timestamp_offset = abs(now_timestamps_selected - current_timestamp_adjust);
                            if timestamp_offset > app.RosbagTimeThreshold / 2
                                valid_check = false;
                                break;
                            end
                        end
                        if ~valid_check
                            currentTime = currentTime + app.RosbagPlayStep;

                            progress = (currentTime - startTime)/(endTime - startTime + 1);
                            waitbar(progress, h, [num2str(floor(progress*100)) '%'], 'Name', 'Saving the topics');
                            continue;
                        end
                        numImageWrite = numImageWrite + 1;
                        str = sprintf('%06d\t\t', numImageWrite);
                        for j = 1 : numTopicsSelected
                            % query the current_arrived_timestamps according to the current_timestamp_adjust
                            now_timestamps = app.current_topics_timestamps{j};
                            now_arrived_timestamps = app.current_topics_arrived_timestamps{j};
                            % find the nearest timestamp index from the current_timestamp_adjust
                            if isequal(topicsSelected{j}, app.RosbagCurrentPointTopic.topic)
                                [~, I] = min(abs(now_timestamps(:) - (current_timestamp_adjust))); % forward the lidar0 100ms
                            else
                                [~, I] = min(abs(now_timestamps(:) - current_timestamp_adjust));
                            end
                            
                            now_arrived_timestamp_selected = now_arrived_timestamps(I);
                            bagid = getBelongtoByTopic(topicsSelected{j}, topicsSelected, bagidSelected);
                            timeOffsetThresholds = app.RosbagTimeThreshold;
                            [messages, timeOffsets, founds] = getSyncMessagesFromTopics(...
                                                                                    app.Rosbag.bags{bagid}, ...
                                                                                    {topicsSelected{j}}, ...
                                                                                    now_arrived_timestamp_selected, ...
                                                                                    timeOffsetThresholds);
                            if founds
                                % judge the type of the message
                                if isequal(messages{1}.MessageType, 'sensor_msgs/Image')
                                    image = readImage(messages{1});
                                    imageName = sprintf('%s/%06d.png', topic_subfolder{j}, numImageWrite);
                                    imwrite(image, imageName);
                                elseif isequal(messages{1}.MessageType, 'sensor_msgs/PointCloud2')
                                    point_xyz = readXYZ(messages{1});
                                    point_intensity = readIntensity(messages{1});
                                    ptCloud = pointCloud(point_xyz, 'Intensity', single(point_intensity));
                                    PointName = sprintf('%s/%06d.pcd', topic_subfolder{j}, numImageWrite);
                                    pcwrite(ptCloud, PointName, 'Encoding', 'binary');
                                else
                                    warningMessageTypeNotSupportedDialog();
                                    return;
                                end
                                message_timestamp_arrive = timeOffsets + now_arrived_timestamp_selected;
                                str = sprintf('%s%.3f\t\t', str, message_timestamp_arrive);
                                % save the timestamp within the message
                                message_header = messages{1}.Header;
                                message_timestamp = message_header.Stamp.Sec + message_header.Stamp.Nsec / 1e9;
                                str = sprintf('%s%.3f\t\t', str, message_timestamp);
                            else
                                str = sprintf('%s%.3f\t\t', str, -1);
                                str = sprintf('%s%.3f\t\t', str, -1);
                            end
                        end
                        str = sprintf('%s\r\n', str);
                        % write the timestamp to the timestamp.txt
                        fprintf(fid, str);

                        currentTime = currentTime + app.RosbagPlayStep;

                        progress = (currentTime - startTime)/(endTime - startTime + 1);
                        waitbar(progress, h, [num2str(floor(progress*100)) '%'], 'Name', 'Saving the topics');
                    elseif isequal(app.query_timestamp_type, 'query_by_arrived_stamp')
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        % query the message by the arrived time
                        all_found = true;
                        object_topic = cell(numTopicsSelected, 5);
                        for j = 1 : numTopicsSelected
                            bagid = getBelongtoByTopic(topicsSelected{j}, topicsSelected, bagidSelected);
                            timeOffsetThresholds = app.RosbagTimeThreshold;
    
                            [messages, ~, ~] = getSyncMessagesFromTopics(...
                                                                        app.Rosbag.bags{bagid}, ...
                                                                        {topicsSelected{j}}, ...
                                                                        currentTime, ...
                                                                        timeOffsetThresholds*2);
    
                            message_ = messages{1};
                            if isempty(message_)
                                all_found = false;
                                break;
                            end
    
                            if isequal(message_.MessageType, 'sensor_msgs/Image')
                                SelectedBag_ = select(app.Rosbag.bags{bagid}, 'Topic', topicsSelected{j});
                                TimeStampImage_ = SelectedBag_.MessageList.Time;
                                if isequal(app.RosbagCurrentTimeBaseType, 'Image')
                                    [message_, timeOffsets] = getMessageTimeNearest(...
                                                                                    SelectedBag_, ...
                                                                                    TimeStampImage_, ...
                                                                                    currentTime, ...
                                                                                    timeOffsetThresholds);
                                elseif isequal(app.RosbagCurrentTimeBaseType, 'Point')
                                    [message_, timeOffsets] = getMessageTimeNearest(...
                                                                                    SelectedBag_, ...
                                                                                    TimeStampImage_, ...
                                                                                    currentTime + app.current_sync_offset, ...
                                                                                    timeOffsetThresholds);
                                else
                                    warningMessageTypeNotSupportedDialog();
                                    return;
                                end
                                if(isempty(message_))
                                    founds = 0;
                                else
                                    founds = 1;
                                end
                            elseif isequal(message_.MessageType, 'sensor_msgs/PointCloud2')
                                SelectedBag_ = select(app.Rosbag.bags{bagid}, 'Topic', topicsSelected{j});
                                TimeStampImage_ = SelectedBag_.MessageList.Time;
                                if isequal(app.RosbagCurrentTimeBaseType, 'Image')
                                    [message_, timeOffsets] = getMessageTimeNearest(...
                                                                                    SelectedBag_, ...
                                                                                    TimeStampImage_, ...
                                                                                    currentTime + app.current_sync_offset, ...
                                                                                    timeOffsetThresholds);
                                elseif isequal(app.RosbagCurrentTimeBaseType, 'Point')
                                    [message_, timeOffsets] = getMessageTimeNearest(...
                                                                                    SelectedBag_, ...
                                                                                    TimeStampImage_, ...
                                                                                    currentTime, ...
                                                                                    timeOffsetThresholds);
                                else
                                    warningMessageTypeNotSupportedDialog();
                                    return;
                                end
                                if(isempty(message_))
                                    founds = 0;
                                else
                                    founds = 1;
                                end
                            else
                                warningMessageTypeNotSupportedDialog();
                                return;
                            end
    
                            if founds
                                % judge the type of the message
                                if isequal(messages{1}.MessageType, 'sensor_msgs/Image')
                                    image = readImage(messages{1});
                                    imageName = sprintf('%s/%06d.png', topic_subfolder{j}, numImageWrite);
                                    object_topic{j, 1} = image;
                                    object_topic{j, 2} = imageName;
                                    object_topic{j, 3} = 'sensor_msgs/Image';
                                elseif isequal(messages{1}.MessageType, 'sensor_msgs/PointCloud2')
                                    point_xyz = readXYZ(messages{1});
                                    point_intensity = readIntensity(messages{1});
                                    ptCloud = pointCloud(point_xyz, 'Intensity', single(point_intensity));
                                    PointName = sprintf('%s/%06d.pcd', topic_subfolder{j}, numImageWrite);
                                    object_topic{j, 1} = ptCloud;
                                    object_topic{j, 2} = PointName;
                                    object_topic{j, 3} = 'sensor_msgs/PointCloud2';
                                else
                                    warningMessageTypeNotSupportedDialog();
                                    return;
                                end
                                message_timestamp_arrive = timeOffsets + currentTime;
                                object_topic{j, 4} = message_timestamp_arrive;
                                % save the timestamp within the message
                                message_header = messages{1}.Header;
                                message_timestamp = message_header.Stamp.Sec + message_header.Stamp.Nsec / 1e9;
                                object_topic{j, 5} = message_timestamp;
                            else
                                all_found = false;
                                break;
                            end
                        end
    
                        if all_found
                            numImageWrite = numImageWrite + 1;
                            str = sprintf('%06d\t\t', numImageWrite);
                            % save the topics
                            for p = 1 : numTopicsSelected
                                object_type = object_topic{p, 3};
                                if isequal(object_type, 'sensor_msgs/Image')
                                    imwrite(object_topic{p, 1}, object_topic{p, 2});
                                elseif isequal(object_type, 'sensor_msgs/PointCloud2')
                                    pcwrite(object_topic{p, 1}, object_topic{p, 2}, 'Encoding', 'binary');
                                end
                                str = sprintf('%s%.3f\t\t', str, object_topic{p, 4});
                                str = sprintf('%s%.3f\t\t', str, object_topic{p, 5});
                            end
                            str = sprintf('%s\r\n', str);
                            % write the timestamp to the timestamp.txt
                            fprintf(fid, str);
                        else
                            % can not sync all topics, skip
                            fprintf('[%f] can not query all sync topics, skip\n', currentTime);
                        end
    
                        currentTime = currentTime + app.RosbagPlayStep;
    
                        progress = (currentTime - startTime)/(endTime - startTime + 1);
                        waitbar(progress, h, [num2str(floor(progress*100)) '%'], 'Name', 'Saving the topics');
                    end
                end
            end
            fclose(fid);
            delete(h);
        else
            warningNoTopicsSelectedDialog();
        end
    else
        warningNoTopicsDialog();
    end

    function waitbar_cancel(src, event)
        %global ifCancel;
        ifCancel = true;
        delete(src);
    end
    function warningNoImageTopicsDialog
        d = dialog('Position', [300 300 250 150], 'Name', 'Warning');
        txt = uicontrol('Parent', d, ...
                        'Style', 'text', ...
                        'Position', [20 80 210 40], ...
                        'HorizontalAlignment', 'center', ...
                        'String', 'None image topics in the rosbag!');
        warndlg('None image topics in the rosbag!', 'No Image Topic in the ROSBAG');
    end
    function tipBaselineTopicDialog(baseline_image_topic)
        str = sprintf('Selected baseline image topic:\r\n%s', baseline_image_topic);
        msgbox(str, 'Tips');
    end
    function warningNoTopicsSelectedDialog
        str = sprintf('None topics selected!\r\n Enable the checkbox in the table!');
        warndlg(str, 'No Topic Selected');
    end

    function warningNoTopicsDialog
        str = sprintf('None topics loaded!\r\n');
        warndlg(str, 'No Topic Loaded');
    end

    function warningNoBaselineSelectedDialog
        str = sprintf('None baseline topics selected!\r\n check your configuration!');
        warndlg(str, 'No Baseline Topic Selected');
    end

    function warningInvalidTopicNameDialog
        str = sprintf('Invalid Topic Name!\r\n topic name should start with /');
        warndlg(str, 'Invalid Topic Name');
    end

    function warningBaselineTopicNotSelectedDialog
        str = sprintf('Baseline topic not selected!\r\n%s', app.RosbagCurrentBaseLineTopic);
        warndlg(str, 'Baseline Topic Not Selected');
    end

    function warningMessageTypeNotSupportedDialog
        str = sprintf('Messages Type not Supported\r\n');
        warndlg(str, 'Invalid Message Type');
    end
    function warningTopicsSelectedConfigChangeDialog
        str = sprintf('Remain the same configuration when extracting timestamps\r\nOr re-extracting');
        warndlg(str, 'TopicsSelectedConfigChange');
    end
    
end  % saveRosbagTopics

%-------------------------------------------------------------------------%
function extractTimestampsFromTopics(app)
    % add your code here
    % get the topics to be save
    table = app.guiStatusTable.mTableSavedTopics.Data;
    if size(table, 1)
        % statistic the selected topics
        numTopicsSelected = 0;
        topicsSelected = {};
        bagidSelected = {};
        for i = 1 : size(table, 1)
            if isequal(table{i, 3}, true)
                numTopicsSelected = numTopicsSelected + 1;
                topicsSelected{numTopicsSelected} = table{i, 1};
                bagidSelected{numTopicsSelected} = table{i, 2};
            end
        end

        app.current_topics_selected = topicsSelected;
        app.current_topics_selected_num = numTopicsSelected;
        if numTopicsSelected
            % select the baseline of the multiple topics
            if isempty(app.RosbagCurrentBaseLineTopic)
                warningNoBaselineSelectedDialog();
                return;
            end
            % check if the baseline topic is selected
            if ~getBelongtoByTopic(app.RosbagCurrentBaseLineTopic, topicsSelected, bagidSelected)
                warningBaselineTopicNotSelectedDialog();
                return;
            end

            % get all the timestamps and the arrived timestamp for the selected topics
            for p = 1 : numTopicsSelected
                current_topic = topicsSelected{p};
                current_bag = app.Rosbag.bags{getBelongtoByTopic(current_topic, topicsSelected, bagidSelected)};
                current_bag = select(current_bag, 'Topic', current_topic);
                current_timeStamps = zeros(current_bag.NumMessages, 1);

                % read the message with bulk mode
                bulk_num = app.bulk_read_num;
                bulk_read_num = floor(current_bag.NumMessages/bulk_num);
                fprintf('begin to extract topic : %s, total topics: %d\n', current_topic, numTopicsSelected);
                for j = 1 : bulk_read_num + 1
                    if j ~= bulk_read_num + 1
                        % buld read the messages
                        bulk_messages = [];
                        bulk_messages = readMessages(current_bag, (1 + (j - 1)*bulk_num):(j*bulk_num));
                        for k = 1 : bulk_num
                            current_bulk_message = bulk_messages{k};
                            current_index = k + (j - 1)*bulk_num;
                            current_timestamp = current_bulk_message.Header.Stamp.Sec + current_bulk_message.Header.Stamp.Nsec / 1e9;
                            current_timeStamps(current_index) = current_timestamp;
                        end
                    else
                        left_to_be_read = mod(current_bag.NumMessages, bulk_num);
                        if left_to_be_read ~= 0
                            bulk_messages = readMessages(current_bag, (bulk_read_num*bulk_num + 1):(current_bag.NumMessages));
                            for k = 1 : left_to_be_read
                                current_bulk_message = bulk_messages{k};
                                current_index = bulk_read_num*bulk_num + k;
                                current_timestamp = current_bulk_message.Header.Stamp.Sec + current_bulk_message.Header.Stamp.Nsec / 1e9;
                                current_timeStamps(current_index) = current_timestamp;
                            end
                        end
                        
                    end
                    fprintf('processing topic: %s, %d/%d\n', current_topic, j, bulk_read_num + 1);
                end
                % save the timestamps
                app.current_topics_timestamps{p} = current_timeStamps;

                current_topics_arrived_timestamps = current_bag.MessageList.Time;
                app.current_topics_arrived_timestamps{p} = current_topics_arrived_timestamps;
            end
            warningExtractionTimestampDoneDialog();
        else
            warningNoTopicsSelectedDialog();
        end
    else
        warningNoTopicsDialog();
    end

    function warningNoTopicsSelectedDialog
        str = sprintf('None topics selected!\r\n Enable the checkbox in the table!');
        warndlg(str, 'No Topic Selected');
    end

    function warningNoTopicsDialog
        str = sprintf('None topics loaded!\r\n');
        warndlg(str, 'No Topic Loaded');
    end

    function warningNoBaselineSelectedDialog
        str = sprintf('None baseline topics selected!\r\n check your configuration!');
        warndlg(str, 'No Baseline Topic Selected');
    end

    function warningBaselineTopicNotSelectedDialog
        str = sprintf('Baseline topic not selected!\r\n%s', app.RosbagCurrentBaseLineTopic);
        warndlg(str, 'Baseline Topic Not Selected');
    end

    function warningExtractionTimestampDoneDialog
        str = sprintf('Timestamps for all selected topics done');
        warndlg(str, 'Extraction done');
    end

end  % extractTimestampsFromTopics

function belongto = getBelongtoByTopic(topic, topics, belongtos)
    belongto = 0;
    for i = 1 : length(topics)
        if isequal(topic, topics{i})
            belongto = belongtos{i};
            break;
        end
    end
end

%-------------------------------------------------------------------------%
function selectSingleShotDir(src, data, app)
    % select the folder for the single shot
    
    % create the sub_directory
    table = app.guiStatusTable.mTableSavedTopics.Data;
    if size(table, 1)
        % statistic the selected topics
        global SSNumTopicsSelected;
        global SSTopicsSelected;
        global SSBagidSelected;

        SSNumTopicsSelected = 0;
        SSTopicsSelected = {};
        SSBagidSelected = {};
        for i = 1 : size(table, 1)
            if isequal(table{i, 3}, true)
                SSNumTopicsSelected = SSNumTopicsSelected + 1;
                SSTopicsSelected{SSNumTopicsSelected} = table{i, 1};
                SSBagidSelected{SSNumTopicsSelected} = table{i, 2};
            end
        end
        if SSNumTopicsSelected
            % select the baseline of the multiple topics
            if isempty(app.RosbagCurrentBaseLineTopic)
                warningNoBaselineSelectedDialog();
                return;
            end
            % check if the baseline topic is selected
            if ~getBelongtoByTopic(app.RosbagCurrentBaseLineTopic, SSTopicsSelected, SSBagidSelected)
                warningBaselineTopicNotSelectedDialog();
                return;
            end

            folder_name = uigetdir(app.SingleShotSaveDir, 'Select folder to save the single shot');
            if isequal(folder_name, 0)
                disp(myLog('user selected cancel'));
            else
                disp(myLog(['user selected ', folder_name]));
                app.SingleShotSaveDir = folder_name;
            end

            % create the subfolders
            global SS_topic_subfolder;
            SS_topic_subfolder = {};
            for i = 1 : SSNumTopicsSelected
                % parse the topic name
                strParse = strsplit(SSTopicsSelected{i}, '/');
                if length(strParse) <= 1
                    warningInvalidTopicNameDialog();
                    return;
                else
                    foldName = strParse{2};
                    SS_topic_subfolder{i} = sprintf('%s/%s', app.SingleShotSaveDir, foldName);
                    mkdir(SS_topic_subfolder{i});
                end  
            end

            % create the timestamp file
            timeStampFile = sprintf('%s/timestamp.txt', app.SingleShotSaveDir);
            global fid;
            fid = fopen(timeStampFile,'w');
            % write the header to the timastamp
            str = sprintf('%s\t', 'ID');
            fprintf(fid, str);
            for i = 1 : SSNumTopicsSelected
                str = sprintf('%s\t', SSTopicsSelected{i});
                fprintf(fid, str);
            end
            fprintf(fid, '\r\n');

            % get the start time and end time for the baseline_topic
            baselineBag = app.Rosbag.bags{getBelongtoByTopic(app.RosbagCurrentBaseLineTopic, SSTopicsSelected, SSBagidSelected)};
            % baselineBag = select(baselineBag, 'Topic', app.RosbagCurrentBaseLineTopic);
            global SSBaseLineStartTime;
            SSBaseLineStartTime = baselineBag.StartTime;

            app.SingleShotSaveDirSet = true;
        else
            warningNoTopicsSelectedDialog();
        end
    else
        warningNoTopicsDialog();
    end

    function warningNoTopicsDialog
        str = sprintf('None topics loaded!\r\n');
        warndlg(str, 'No Topic Loaded');
    end

    function warningNoTopicsSelectedDialog
        str = sprintf('None topics selected!\r\n Enable the checkbox in the table!');
        warndlg(str, 'No Topic Selected');
    end

    function warningInvalidTopicNameDialog
        str = sprintf('Invalid Topic Name!\r\n topic name should start with /');
        warndlg(str, 'Invalid Topic Name');
    end

    function warningNoBaselineSelectedDialog
        str = sprintf('None baseline topics selected!\r\n check your configuration!');
        warndlg(str, 'No Baseline Topic Selected');
    end

    function warningBaselineTopicNotSelectedDialog
        str = sprintf('Baseline topic not selected!\r\n%s', app.RosbagCurrentBaseLineTopic);
        warndlg(str, 'Baseline Topic Not Selected');
    end

end  % selectSingleShotDir

%-------------------------------------------------------------------------%
function SingleShotImage(src, data, app)
    % shot the current of the selected topics
    if app.SingleShotSaveDirSet
        % save your selected topics by current time
        %%%%%%%%%%% shot the current selected topics %%%%%%%%%%%%%%
        global SSNumTopicsSelected;
        global SSTopicsSelected;
        global SSBagidSelected;

        global SSBaseLineStartTime;

        currentTime = SSBaseLineStartTime + app.RosbagPlaySliderCurrentTime;

        str = sprintf('%06d\t\t', app.SingleShotIndex);
        for j = 1 : SSNumTopicsSelected
            bagid = getBelongtoByTopic(SSTopicsSelected{j}, SSTopicsSelected, SSBagidSelected);
            timeOffsetThresholds = app.RosbagTimeThreshold;

            [messages, ~, ~] = getSyncMessagesFromTopics(...
                                                                    app.Rosbag.bags{bagid}, ...
                                                                    {SSTopicsSelected{j}}, ...
                                                                    currentTime, ...
                                                                    timeOffsetThresholds + 0.1 );

            message_ = messages{1};
            if isequal(message_.MessageType, 'sensor_msgs/Image')
                SelectedBag_ = select(app.Rosbag.bags{bagid}, 'Topic', SSTopicsSelected{j});
                TimeStampImage_ = SelectedBag_.MessageList.Time;
                if isequal(app.RosbagCurrentTimeBaseType, 'Image')
                    [message_, timeOffsets] = getMessageTimeNearest(...
                                                                    SelectedBag_, ...
                                                                    TimeStampImage_, ...
                                                                    currentTime, ...
                                                                    timeOffsetThresholds);
                elseif isequal(this.RosbagCurrentTimeBaseType, 'Point')
                    [message_, timeOffsets] = getMessageTimeNearest(...
                                                                    SelectedBag_, ...
                                                                    TimeStampImage_, ...
                                                                    currentTime + app.current_sync_offset, ...
                                                                    timeOffsetThresholds);
                else
                    warningMessageTypeNotSupportedDialog();
                    return;
                end
                if(isempty(message_))
                    founds = 0;
                else
                    founds = 1;
                end
            elseif isequal(message_.MessageType, 'sensor_msgs/PointCloud2')
                SelectedBag_ = select(app.Rosbag.bags{bagid}, 'Topic', SSTopicsSelected{j});
                TimeStampImage_ = SelectedBag_.MessageList.Time;
                if isequal(app.RosbagCurrentTimeBaseType, 'Image')
                    [message_, timeOffsets] = getMessageTimeNearest(...
                                                                    SelectedBag_, ...
                                                                    TimeStampImage_, ...
                                                                    currentTime + app.current_sync_offset, ...
                                                                    timeOffsetThresholds);
                elseif isequal(this.RosbagCurrentTimeBaseType, 'Point')
                    [message_, timeOffsets] = getMessageTimeNearest(...
                                                                    SelectedBag_, ...
                                                                    TimeStampImage_, ...
                                                                    currentTime, ...
                                                                    timeOffsetThresholds);
                else
                    warningMessageTypeNotSupportedDialog();
                    return;
                end
                if(isempty(message_))
                    founds = 0;
                else
                    founds = 1;
                end
            else
                warningMessageTypeNotSupportedDialog();
                return;
            end

            if founds
                % judge the type of the message
                global SS_topic_subfolder;
                if isequal(message_.MessageType, 'sensor_msgs/Image')
                    image = readImage(message_);
                    imageName = sprintf('%s/%06d.png', SS_topic_subfolder{j}, app.SingleShotIndex);
                    imwrite(image, imageName);
                elseif isequal(message_.MessageType, 'sensor_msgs/PointCloud2')
                    point_xyz = readXYZ(message_);
                    point_intensity = readIntensity(message_);
                    ptCloud = pointCloud(point_xyz, 'Intensity', single(point_intensity));
                    PointName = sprintf('%s/%06d.pcd', SS_topic_subfolder{j}, app.SingleShotIndex);
                    pcwrite(ptCloud, PointName, 'Encoding', 'binary');
                else
                    warningMessageTypeNotSupportedDialog();
                    return;
                end
                timeOffsets = timeOffsets + currentTime;
                str = sprintf('%s%.3f\t\t', str, timeOffsets);
            else
                str = sprintf('%s%.3f\t\t', str, -1);
            end
        end
        str = sprintf('%s\r\n', str);
        % write the timestamp to the timestamp.txt
        global fid;
        fprintf(fid, str);

        app.SingleShotIndex = app.SingleShotIndex + 1;
    else
        str = sprintf('set the save dir before single shot!');
        warndlg(str, 'Saving directory not specified');
    end

    function warningMessageTypeNotSupportedDialog
        str = sprintf('Messages Type not Supported\r\n');
        warndlg(str, 'Invalid Message Type');
    end
    
end  % SingleShotImage

%-------------------------------------------------------------------------%
function ResetSingleShot(src, data, app)
    % reset the single shot
    app.SingleShotIndex = 1;
end  % ResetSingleShot



