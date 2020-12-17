classdef ros_data_multiBag_image_point < handle
    % ros_data_multiBag_image_point() opens a toolstrip with docked figures to view the rosdata 
    % and support sevel operations, especilly for the multi-Rosbag operation and enable the joint
    % operation of image and point
    %
    % Example:
    %   Run "ros_data_multiBag_image_point()"

    % Author(s): ethan
    % Copyright ccyinlu@whu.edu.cn
    % Date: 20190102

    properties (Transient = true)
        ToolGroup
        
        guiRosbagTable
        guiStatusTable
        guiImageView
        guiPointView
        guiPlay

        current_dir
        RosbagFileSaveDir
        SingleShotSaveDir
        SingleShotSaveDirSet
        SingleShotIndex

        % will be cell type
        Rosbag % struct
        RosbagCurrentFileName

        % will be cell type
        RosbagAvailableImageTopics
        RosbagAvailablePointTopics

        % will be cell type
        current_topics_timestamps
        current_topics_arrived_timestamps
        current_topics_selected
        current_topics_selected_num

        RosbagCurrentImageTopic
        RosbagCurrentPointTopic
        RosbagCurrentTimeBaseType
        RosbagCurrentBaseLineTopic

        current_image
        current_point
        current_point_intensity
        current_point_view
        current_point_lim
        current_point_Ilim
        current_point_size
        current_point_zoom_flag
        current_point_render_type
        current_syncStatus

        current_sync_offset

        query_timestamp_type
        bulk_read_num

        defaultBackGroundColor

        RosbagPlayInterval
        RosbagPlayStep
        RosbagTimeThreshold
        RosbagPlaySliderStartTime
        RosbagPlaySliderCurrentTime
        RosbagPlaySliderEndTime
        RosbagPlaySliderMaxTime

        dataCursorImageHandle
        dataCursorPointHandle
    end

    methods

        function this = ros_data_multiBag_image_point(varargin)
            % create tool group
            import matlab.ui.internal.toolstrip.*  % for convenience below
            % set the path
            current_file_path = fileparts(which('ros_data_multiBag_image_point.m'));
            addpath(fullfile(current_file_path, './utils/interface'));
            addpath(fullfile(current_file_path, './utils/display'));
            addpath(fullfile(current_file_path, './figure'));
            addpath(fullfile(current_file_path, './callback'));

            %init the properties
            init(this);

            % create tool group
            this.ToolGroup = matlab.ui.internal.desktop.ToolGroup('Multi-Rosbag Image & Point');
            addlistener(this.ToolGroup, 'GroupAction',@(src, event) closeCallback(this, event));
            % hidden the data browser
            this.ToolGroup.disableDataBrowser();


            % create figure
            this.guiStatusTable = createMultiRosbagStatusTableFigure(this);
            this.guiRosbagTable = createMultiRosbagTableFigure(this);
            this.guiImageView   = createMultiRosbagImageViewFigure(this);
            this.guiPointView   = createMultiRosbagPointViewFigure(this);
            this.guiPlay        = createMultiRosbagPlayFigure(this);
            emptyFigure1        = createEmptyFigure(this);
            emptyFigure2        = createEmptyFigure(this);
            emptyFigure3        = createEmptyFigure(this);
            emptyFigure4        = createEmptyFigure(this);
            
            % create tab group 
            tabgroup = ros_data_multiBag_image_point_buildTabGroup(this);
            % add tab group to toolstrip (via tool group api)
            this.ToolGroup.addTabGroup(tabgroup);
            % select current tab (via tool group api)
            this.ToolGroup.SelectedTab = 'tabRosbag';
            % render app
            this.ToolGroup.setPosition(100,100,1600,820);
            this.ToolGroup.open;
            
            % add plot as a document
            this.ToolGroup.addFigure(this.guiStatusTable.handle);
            this.ToolGroup.addFigure(this.guiRosbagTable.handle);
            this.ToolGroup.addFigure(this.guiImageView.handle);
            this.ToolGroup.addFigure(this.guiPointView.handle);
            this.ToolGroup.addFigure(this.guiPlay.handle);
            this.ToolGroup.addFigure(emptyFigure1.handle);
            this.ToolGroup.addFigure(emptyFigure2.handle);
            this.ToolGroup.addFigure(emptyFigure3.handle);
            this.ToolGroup.addFigure(emptyFigure4.handle);
            
            this.guiStatusTable.handle.Visible  = 'on';
            this.guiRosbagTable.handle.Visible  = 'on';
            emptyFigure1.handle.Visible         = 'on';
            emptyFigure2.handle.Visible         = 'on';
            this.guiImageView.handle.Visible    = 'on';
            this.guiPointView.handle.Visible    = 'on';
            emptyFigure3.handle.Visible         = 'on';
            this.guiPlay.handle.Visible         = 'on';
            emptyFigure4.handle.Visible         = 'on';

            initFigure(this);

            setDefaultLayout(this);

            % store java toolgroup so that app will stay in memory
            internal.setJavaCustomData(this.ToolGroup.Peer, this);

        end

        function delete(this)
            if ~isempty(this.ToolGroup) && isvalid(this.ToolGroup)
                delete(this.ToolGroup);
            end

            if ~isempty(this.guiRosbagTable.handle) && isvalid(this.guiRosbagTable.handle)
                delete(this.guiRosbagTable);
            end
            if ~isempty(this.guiStatusTable.handle) && isvalid(this.guiStatusTable.handle)
                delete(this.guiStatusTable);
            end
            if ~isempty(this.guiImageView.handle) && isvalid(this.guiImageView.handle)
                delete(this.guiImageView);
            end
            if ~isempty(this.guiPointView.handle) && isvalid(this.guiPointView.handle)
                delete(this.guiPointView);
            end
            if ~isempty(this.guiPlay.handle) && isvalid(this.guiPlay.handle)
                delete(this.guiPlay);
            end
        end

        function closeCallback(this, event)
            ET = event.EventData.EventType;
            if strcmp(ET, 'CLOSED')
                delete(this);
            end
        end

        function setDefaultLayout(this)
            % default document layout
            MD = com.mathworks.mlservices.MatlabDesktopServices.getDesktop;
            pause(0.01);
            MD.setDocumentArrangement(this.ToolGroup.Name, 2, java.awt.Dimension(3,3));
            MD.setDocumentColumnWidths(this.ToolGroup.Name, [0.2 0.4 0.4]);
            MD.setDocumentRowHeights(this.ToolGroup.Name, [0.2 0.6 0.2]);
            MD.setDocumentRowSpan(this.ToolGroup.Name, 0, 0, 3);
            MD.setDocumentColumnSpan(this.ToolGroup.Name, 0, 1, 2);
            MD.setDocumentColumnSpan(this.ToolGroup.Name, 2, 1, 2);
        end

        function init(this)
            this.guiRosbagTable = [];
            this.guiStatusTable = [];
            this.guiImageView = [];
            this.guiPointView = [];
            this.guiPlay = [];

            % init the info
            initInfo(this);

            % suppress all the warnings
            warning('off');
        end

        function initInfo(this)
            this.current_dir = fileparts(which('ros_data_multiBag_image_point.m'));

            this.Rosbag = struct();
            this.Rosbag.numBags = 0;
            this.Rosbag.bags = {};
            this.Rosbag.filenames = {};
            this.Rosbag.availableTopics = {};
            this.Rosbag.numTopics = {};
            this.Rosbag.enable = {};
            this.Rosbag.startTime = {};
            this.Rosbag.endTime = {};
            this.Rosbag.duration = {};
            this.Rosbag.numMessages = {};


            this.RosbagCurrentFileName = this.current_dir;
            this.RosbagFileSaveDir = this.current_dir;
            this.SingleShotSaveDir = this.current_dir;
            this.SingleShotSaveDirSet = false;

            this.SingleShotIndex = 1;
            
            this.RosbagAvailableImageTopics = struct();
            this.RosbagAvailableImageTopics.topics = {};
            this.RosbagAvailableImageTopics.belongTo = {};
            this.RosbagAvailableImageTopics.count = 0;
            this.RosbagAvailablePointTopics = struct();
            this.RosbagAvailablePointTopics.topics = {};
            this.RosbagAvailablePointTopics.belongTo = {};
            this.RosbagAvailablePointTopics.count = 0;

            this.RosbagCurrentImageTopic = struct();
            this.RosbagCurrentImageTopic.topic = [];
            this.RosbagCurrentImageTopic.belongTo = [];
            this.RosbagCurrentPointTopic = struct();
            this.RosbagCurrentPointTopic.topic = [];
            this.RosbagCurrentPointTopic.belongTo = [];
            this.current_point_view = struct();
            this.current_point_view.az = -145;
            this.current_point_view.el = 52;

            % specify the current [xLimMin xLimMax yLimMin yLimMax zLimMin zLimMax]
            this.current_point_lim = [-100 100 -100 100 -20 20];
            this.current_point_Ilim = [0 255];

            this.current_sync_offset = 0;

            this.current_point_zoom_flag = false;

            this.current_point_size = 1;
            this.current_point_render_type = 'range'

            this.RosbagCurrentTimeBaseType = 'Image';

            this.RosbagCurrentBaseLineTopic = '';

            this.current_image = [];
            this.current_point = [];
            this.current_point_intensity = [];
            this.current_syncStatus = false;

            % this.query_timestamp_type = 'query_by_messageIn_stamp';
            this.query_timestamp_type = 'query_by_arrived_stamp';
            this.bulk_read_num = 50;

            this.RosbagPlayStep = 0.1;
            this.RosbagPlayInterval = 0.0;
            this.RosbagTimeThreshold = 0.1;
            
            this.RosbagPlaySliderStartTime = 0.0;
            this.RosbagPlaySliderCurrentTime = 0.0;
            this.RosbagPlaySliderEndTime = 0.0;
            this.RosbagPlaySliderMaxTime = 0.0;

            this.defaultBackGroundColor = [0.25 0.25 0.25];

            this.dataCursorImageHandle = [];
            this.dataCursorPointHandle = [];

            this.current_topics_timestamps = [];
            this.current_topics_arrived_timestamps = [];
            this.current_topics_selected = [];
            this.current_topics_selected_num = 0;
        end

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
        end

        function tableData = generateRosbagTableData(this)
            tableData = cell(this.Rosbag.numBags, 4);
            for i = 1 : this.Rosbag.numBags
                tableData{i, 1} = this.Rosbag.filenames{i};
                tableData{i, 2} = this.Rosbag.availableTopics{i}{1};
                tableData{i, 3} = this.Rosbag.numTopics{i};
                tableData{i, 4} = this.Rosbag.enable{i};
            end
        end

        function tableData = generateSavedTopicsTableData(this)
            availableImageTopicsNum = this.RosbagAvailableImageTopics.count;
            availablePointTopicsNum = this.RosbagAvailablePointTopics.count;
            tableData = cell(availableImageTopicsNum + availablePointTopicsNum, 3);
            if availableImageTopicsNum
                % update the images topics
                for i = 1 : availableImageTopicsNum
                    tableData{i, 1} = this.RosbagAvailableImageTopics.topics{i};
                    tableData{i, 2} = this.RosbagAvailableImageTopics.belongTo{i};
                    tableData{i, 3} = false;
                end
            end
            if availablePointTopicsNum
                % update the topics topics
                for i = 1 : availablePointTopicsNum
                    tableData{i + availableImageTopicsNum, 1} = this.RosbagAvailablePointTopics.topics{i};
                    tableData{i + availableImageTopicsNum, 2} = this.RosbagAvailablePointTopics.belongTo{i};
                    tableData{i + availableImageTopicsNum, 3} = false;
                end
            end
        end

        function updateAvailableImageTopics(this)
            this.RosbagAvailableImageTopics.topics = {};
            this.RosbagAvailableImageTopics.belongTo = {};
            this.RosbagAvailableImageTopics.count = 0;

            for i = 1 : this.Rosbag.numBags
                topicsCell = getTopicsByType(this.Rosbag.bags{i}, 'sensor_msgs/Image');
                topicsCell = [topicsCell getTopicsByType(this.Rosbag.bags{i}, 'sensor_msgs/CompressedImage')];
                if ~isempty(topicsCell)
                    for j = 1 : length(topicsCell)
                        if ~sum(ismember(this.RosbagAvailableImageTopics.topics, topicsCell{j}))
                            this.RosbagAvailableImageTopics.count = this.RosbagAvailableImageTopics.count + 1;
                            this.RosbagAvailableImageTopics.topics{this.RosbagAvailableImageTopics.count} = topicsCell{j};
                            this.RosbagAvailableImageTopics.belongTo{this.RosbagAvailableImageTopics.count} = i;
                        end
                    end
                end
            end
 
        end

        function updateAvailableImageTopicsView(this, index)
            if this.RosbagAvailableImageTopics.count
                if  index <= this.RosbagAvailableImageTopics.count
                    tableData = cell(1 ,2);
                    tableData{1, 1} = this.RosbagAvailableImageTopics.topics{index};
                    tableData{1, 2} = this.RosbagAvailableImageTopics.belongTo{index};
                    this.guiStatusTable.mTableImageTopics.ColumnFormat{1} = this.RosbagAvailableImageTopics.topics;
                    this.guiStatusTable.mTableImageTopics.Data = tableData;
                end
            else
                tableData = {};
                this.guiStatusTable.mTableImageTopics.Data = tableData;
            end
        end

        function updateCurrentImageTopic(this, index)
            if this.RosbagAvailableImageTopics.count 
                if index <= this.RosbagAvailableImageTopics.count
                    this.RosbagCurrentImageTopic.topic = this.RosbagAvailableImageTopics.topics{index};
                    this.RosbagCurrentImageTopic.belongTo = this.RosbagAvailableImageTopics.belongTo{index};
                end
            else
                this.RosbagCurrentImageTopic.topic = [];
                this.RosbagCurrentImageTopic.belongTo = [];
            end
        end

        function updateAvailablePointTopics(this)
            this.RosbagAvailablePointTopics.topics = {};
            this.RosbagAvailablePointTopics.belongTo = {};
            this.RosbagAvailablePointTopics.count = 0;
            for i = 1 : this.Rosbag.numBags
                topicsCell = getTopicsByType(this.Rosbag.bags{i}, 'sensor_msgs/PointCloud2');
                if ~isempty(topicsCell)
                    for j = 1 : length(topicsCell)
                        if ~sum(ismember(this.RosbagAvailablePointTopics.topics, topicsCell{j}))
                            this.RosbagAvailablePointTopics.count = this.RosbagAvailablePointTopics.count + 1;
                            this.RosbagAvailablePointTopics.topics{this.RosbagAvailablePointTopics.count} = topicsCell{j};
                            this.RosbagAvailablePointTopics.belongTo{this.RosbagAvailablePointTopics.count} = i;
                        end
                    end
                end
            end
        end

        function updateCurrentPointTopic(this, index)
            if this.RosbagAvailablePointTopics.count 
                if index <= this.RosbagAvailablePointTopics.count
                    this.RosbagCurrentPointTopic.topic = this.RosbagAvailablePointTopics.topics{index};
                    this.RosbagCurrentPointTopic.belongTo = this.RosbagAvailablePointTopics.belongTo{index};
                end
            else
                this.RosbagCurrentPointTopic.topic = [];
                this.RosbagCurrentPointTopic.belongTo = [];
            end
        end

        function updateAvailablePointTopicsView(this, index)
            if this.RosbagAvailablePointTopics.count
                if index <= this.RosbagAvailablePointTopics.count
                    tableData = cell(1 ,2);
                    tableData{1, 1} = this.RosbagAvailablePointTopics.topics{index};
                    tableData{1, 2} = this.RosbagAvailablePointTopics.belongTo{index};
                    this.guiStatusTable.mTablePointTopics.ColumnFormat{1} = this.RosbagAvailablePointTopics.topics;
                    this.guiStatusTable.mTablePointTopics.Data = tableData;
                end
            else
                tableData = {};
                this.guiStatusTable.mTablePointTopics.Data = tableData;
            end
        end

        function tableData = generateStatusImageTableData(this)
            tableData = cell(this.Rosbag.numBags, 4);
            for i = 1 : this.Rosbag.numBags
                tableData{i, 1} = this.Rosbag.filenames{i};
                tableData{i, 2} = this.Rosbag.availableTopics{i}{1};
                tableData{i, 3} = this.Rosbag.numTopics{i};
                tableData{i, 4} = this.Rosbag.enable{i};
            end
        end

        function updatePlayView(this)
            % update the guiPlay edit
            set(this.guiPlay.mEditBoxStartTime,'String', sprintf('%.3f', this.RosbagPlaySliderStartTime));
            set(this.guiPlay.mEditBoxCurrentTime,'String', sprintf('%.3f', this.RosbagPlaySliderCurrentTime));
            set(this.guiPlay.mEditBoxEndTime,'String', sprintf('%.3f', this.RosbagPlaySliderEndTime));
            set(this.guiPlay.mTextBoxMaxTime,'String', sprintf('%.3f', this.RosbagPlaySliderMaxTime));
            % update the play slider
            set(this.guiPlay.mSliderBoxPlay,'Min', this.RosbagPlaySliderStartTime);
            set(this.guiPlay.mSliderBoxPlay,'Max', this.RosbagPlaySliderEndTime);
            set(this.guiPlay.mSliderBoxPlay,'Value', this.RosbagPlaySliderCurrentTime);
            if this.RosbagPlaySliderStartTime ~= this.RosbagPlaySliderEndTime
                set(this.guiPlay.mSliderBoxPlay,'SliderStep', [0.1/(this.RosbagPlaySliderEndTime - this.RosbagPlaySliderStartTime) 0.1/(this.RosbagPlaySliderEndTime - this.RosbagPlaySliderStartTime)]);
            else
                set(this.guiPlay.mSliderBoxPlay,'SliderStep', [0 0]);
            end
        end

        function updatePlayInfo(this)
            updateFlag = false;
            if isequal(this.RosbagCurrentTimeBaseType, 'Image')
                if ~isempty(this.RosbagCurrentImageTopic.topic)
                    curRosbag = this.Rosbag.bags{this.RosbagCurrentImageTopic.belongTo};
                    updateFlag = true; 
                end
            elseif isequal(this.RosbagCurrentTimeBaseType, 'Point')
                if ~isempty(this.RosbagCurrentPointTopic.topic)
                    curRosbag = this.Rosbag.bags{this.RosbagCurrentPointTopic.belongTo};
                    updateFlag = true; 
                end
            end
            if updateFlag
                this.RosbagPlaySliderStartTime = 0.0;
                this.RosbagPlaySliderCurrentTime = 0.0;
                this.RosbagPlaySliderEndTime = curRosbag.EndTime - curRosbag.StartTime;
                this.RosbagPlaySliderMaxTime = curRosbag.EndTime - curRosbag.StartTime;
            end
        end

        function updateImagePoint(this)
            image_found = false;
            point_found = false;
            if isequal(this.RosbagCurrentTimeBaseType, 'Image')
                if ~isempty(this.RosbagCurrentImageTopic.topic)
                    this.RosbagCurrentBaseLineTopic = this.RosbagCurrentImageTopic.topic;
                    curRosbagImage = this.Rosbag.bags{this.RosbagCurrentImageTopic.belongTo};
                    rosbagStartTime = curRosbagImage.StartTime;
                    % update the current image
                    currentSelectedBag = select(curRosbagImage, 'Topic', this.RosbagCurrentImageTopic.topic);
                    currentTimeStampImage = currentSelectedBag.MessageList.Time;
                    [message, ~] = getMessageTimeNearest(...
                                                    currentSelectedBag, ...
                                                    currentTimeStampImage, ...
                                                    this.RosbagPlaySliderCurrentTime + rosbagStartTime, ...
                                                    this.RosbagTimeThreshold);
                    if ~isempty(message)
                        this.current_image = readImage(message);
                        image_found = true;
                    else
                        this.current_image = [];
                        image_found = false;
                    end
                    this.updateImageView();

                    % update the current point
                    if ~isempty(this.RosbagCurrentPointTopic.topic)
                        curRosbagPoint = this.Rosbag.bags{this.RosbagCurrentPointTopic.belongTo};
                        % update the current point
                        currentSelectedBag = select(curRosbagPoint, 'Topic', this.RosbagCurrentPointTopic.topic);
                        currentTimeStampPoint = currentSelectedBag.MessageList.Time;
                        [message, ~] = getMessageTimeNearest(...
                                                        currentSelectedBag, ...
                                                        currentTimeStampPoint, ...
                                                        this.RosbagPlaySliderCurrentTime + rosbagStartTime + this.current_sync_offset, ...
                                                        this.RosbagTimeThreshold);
                        if ~isempty(message)
                            this.current_point = readXYZ(message);
                            this.current_point_intensity = readIntensity(message);
                            point_found = true;
                        else
                            this.current_point = [];
                            point_found = false;
                        end
                        this.updatePointView();
                    else
                        this.current_point = [];
                        point_found = false;
                        this.updatePointView();
                    end
                else
                    this.RosbagCurrentBaseLineTopic = '';
                    this.current_image = [];
                    this.updateImageView();

                    this.current_point = [];
                    this.updatePointView();
                end
            elseif isequal(this.RosbagCurrentTimeBaseType, 'Point')
                if ~isempty(this.RosbagCurrentPointTopic.topic)
                    this.RosbagCurrentBaseLineTopic = this.RosbagCurrentPointTopic.topic;
                    curRosbagPoint = this.Rosbag.bags{this.RosbagCurrentPointTopic.belongTo};
                    rosbagStartTime = curRosbagPoint.StartTime;
                    % update the current point
                    currentSelectedBag = select(curRosbagPoint, 'Topic', this.RosbagCurrentPointTopic.topic);
                    currentTimeStampPoint = currentSelectedBag.MessageList.Time;
                    [message, ~] = getMessageTimeNearest(...
                                                    currentSelectedBag, ...
                                                    currentTimeStampPoint, ...
                                                    this.RosbagPlaySliderCurrentTime + rosbagStartTime, ...
                                                    this.RosbagTimeThreshold);
                    if ~isempty(message)
                        this.current_point = readXYZ(message);
                        this.current_point_intensity = readIntensity(message);
                        point_found = true;
                    else
                        this.current_point = [];
                        point_found = false;
                    end
                    this.updatePointView();

                    % update the current image
                    if ~isempty(this.RosbagCurrentImageTopic.topic)
                        curRosbagImage = this.Rosbag.bags{this.RosbagCurrentImageTopic.belongTo};
                        % update the current image
                        currentSelectedBag = select(curRosbagImage, 'Topic', this.RosbagCurrentImageTopic.topic);
                        currentTimeStampImage = currentSelectedBag.MessageList.Time;
                        [message, ~] = getMessageTimeNearest(...
                                                        currentSelectedBag, ...
                                                        currentTimeStampImage, ...
                                                        this.RosbagPlaySliderCurrentTime + rosbagStartTime + this.current_sync_offset, ...
                                                        this.RosbagTimeThreshold);
                        if ~isempty(message)
                            this.current_image = readImage(message);
                            image_found = true;
                        else
                            this.current_image = [];
                            image_found = false;
                        end
                        this.updateImageView();
                    else
                        this.current_image = [];
                        image_found = false;
                        this.updateImageView();
                    end
                else
                    this.RosbagCurrentBaseLineTopic = '';
                    this.current_image = [];
                    this.updateImageView();

                    this.current_point = [];
                    this.updatePointView();
                end
            end
            % update the syncStatus
            this.current_syncStatus = image_found & point_found;
            updateSyncStatus(this);
            drawnow;
        end

        function updateImageView(this)
            % show the this.current_image to the this.guiImageView.handle
            if ~isempty(this.current_image)
                % figure(this.guiImageView.handle);
                % axes(this.guiImageView.axesImageView);
                imshow(this.current_image, 'Parent', this.guiImageView.axesImageView);
                % set(gcf, 'Color', this.defaultBackGroundColor)
                % drawnow;
                if this.RosbagPlayInterval ~= 0.0
                    pause(this.RosbagPlayInterval);
                end
            else
                cla(this.guiImageView.axesImageView);
            end
        end

        function updatePointView(this)
            % show the this.current_point to the this.guiPointView.handle
            if ~isempty(this.current_point)
                % figure(this.guiPointView.handle);
                % axes(this.guiPointView.axesPointView);
                pointsShow(this.guiPointView.axesPointView, ...
                           [this.current_point double(this.current_point_intensity)], ...
                           this.current_point_lim, ...
                           this.current_point_Ilim, ...
                           this.current_point_render_type, ...
                           this.current_point_size);
                view(this.guiPointView.axesPointView, this.current_point_view.az, this.current_point_view.el);
                % drawnow;
                if this.RosbagPlayInterval ~= 0.0
                    pause(this.RosbagPlayInterval);
                end
            else
                cla(this.guiPointView.axesPointView); 
            end
        end

        function updateSyncStatus(this)
            images={...
                    fullfile(this.current_dir, 'icons/ok.png'), ...
                    fullfile(this.current_dir, 'icons/fail.png')};
            buttons={...
                    this.guiPlay.mPushbuttonBoxSyncStatus};
    
            bgColor = [0.94 0.94 0.94];
    
            % redraw the syncStatus
            if this.current_syncStatus
                image = imread(images{1}, 'BackGroundColor', bgColor);
            else
                image = imread(images{2}, 'BackGroundColor', bgColor);
            end
            buttons{1}.Units='pixels';
            image = imresize(image, fliplr(buttons{1}.Position(1,3:4)));
            buttons{1}.Units='normalized';
            buttons{1}.CData=image;
        end
    end
end


