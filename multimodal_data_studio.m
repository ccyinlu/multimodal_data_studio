function multimodal_data_studio()
	% multimodal_data_studio supporting the following functions:
	% 1: multi_rosbag tool for image and lidar points operation
  % 2: camera_lidar_calibration_toolbox based on correspoding planes matching using chessboard
  

	
	gui = struct();

  % Open a window and add some menus
  gui.handle = figure( ...
      'Name', 'multimodal_data_studio', ...
      'NumberTitle', 'off', ...
      'MenuBar', 'none', ...
      'Toolbar', 'none', ...
      'HandleVisibility', 'on', ...
      'Visible', 'on', ...
  'Resize', 'off');
	
	figureWidth = 640;
	figureHeight = 480;
	BoxWidth = 0.2;
  BoxHeight = BoxWidth * figureWidth / figureHeight;
  BoxLeftMargin = 0.2;
	BoxHMargin = 0.2;
	BoxVMargin = 0.2;
  BoxVPos = 0.6;
	
	set(gui.handle, 'Position', [0 0 figureWidth figureHeight]);
	
	movegui(gui.handle, 'center');
    
	bgColor = [0.94 0.94 0.94];
	% set the path
	current_file_path = fileparts(which('multimodal_data_studio.m'));
	addpath(fullfile(current_file_path, './utils/interface'));
	multi_rosbag_image_filename = fullfile(current_file_path, './icons/multi_rosbag.png');
	chessboard_camera_lidar_calibration_image_filename = fullfile(current_file_path, './icons/chessboard_camera_lidar_calibration.png');
	chessboard_camera_calibration_image_filename = fullfile(current_file_path, './icons/chessboard_camera_calibration.png');
	
	% create the pushbutton play control for multi_rosbag
  gui.mPushbuttonMultiRosbagLocation = [BoxLeftMargin + 0*(BoxHMargin + BoxWidth) BoxVPos BoxWidth BoxHeight];
  gui.mPushbuttonMultiRosbag = uicontrol('Parent', gui.handle, 'style', 'pushbutton', 'Units', 'normalized', 'Position', gui.mPushbuttonMultiRosbagLocation);
  % jh = findjobj(gui.mPushbuttonMultiRosbag);
  % jh.setFlyOverAppearance(true); % set the button boarder invisible
  set(gui.mPushbuttonMultiRosbag, 'callback', @onButtonMultiRosbag);
	
	% draw the multi_rosbag button image
	image = imread(multi_rosbag_image_filename, 'BackGroundColor', bgColor);
	gui.mPushbuttonMultiRosbag.Units='pixels';
	image = imresize(image, fliplr(gui.mPushbuttonMultiRosbag.Position(1,3:4))); % resize the png to 0.8 of the button boarder
	gui.mPushbuttonMultiRosbag.Units='normalized';
	gui.mPushbuttonMultiRosbag.CData=image;

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% create the pushbutton for camera extrinsic calibration
  gui.mPushbuttonChessboardCameraLidarCalibrationLocation = [BoxLeftMargin + 1*(BoxHMargin + BoxWidth) BoxVPos BoxWidth BoxHeight];
  gui.mPushbuttonChessboardCameraLidarCalibration = uicontrol('Parent', gui.handle, 'style', 'pushbutton', 'Units', 'normalized', 'Position', gui.mPushbuttonChessboardCameraLidarCalibrationLocation);
  % jh = findjobj(gui.mPushbuttonChessboardCameraLidarCalibration);
  % jh.setFlyOverAppearance(true); % set the button boarder invisible
  set(gui.mPushbuttonChessboardCameraLidarCalibration, 'callback', @onButtonChessboardCameraLidarCalibration);
	
	% draw the cameara-lidar calibration button image
	image = imread(chessboard_camera_lidar_calibration_image_filename, 'BackGroundColor', bgColor);
	gui.mPushbuttonChessboardCameraLidarCalibration.Units = 'pixels';
	image = imresize(image, fliplr(gui.mPushbuttonChessboardCameraLidarCalibration.Position(1,3:4))); % resize the png to 0.8 of the button boarder
	gui.mPushbuttonChessboardCameraLidarCalibration.Units = 'normalized';
	gui.mPushbuttonChessboardCameraLidarCalibration.CData = image;

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% create the pushbutton for camera intrinsic calibration
  gui.mPushbuttonChessboardCameraCalibrationLocation = [BoxLeftMargin + 0*(BoxHMargin + BoxWidth) BoxVPos - 1*(BoxHeight + BoxVMargin) BoxWidth BoxHeight];
  gui.mPushbuttonChessboardCameraCalibration = uicontrol('Parent', gui.handle, 'style', 'pushbutton', 'Units', 'normalized', 'Position', gui.mPushbuttonChessboardCameraCalibrationLocation);
  % jh = findjobj(gui.mPushbuttonChessboardCameraLidarCalibration);
  % jh.setFlyOverAppearance(true); % set the button boarder invisible
  set(gui.mPushbuttonChessboardCameraCalibration, 'callback', @onButtonChessboardCameraCalibration);
	
	% draw the camera-calibration button image
	image = imread(chessboard_camera_calibration_image_filename, 'BackGroundColor', bgColor);
	gui.mPushbuttonChessboardCameraCalibration.Units = 'pixels';
	image = imresize(image, fliplr(gui.mPushbuttonChessboardCameraCalibration.Position(1,3:4))); % resize the png to 0.8 of the button boarder
	gui.mPushbuttonChessboardCameraCalibration.Units = 'normalized';
	gui.mPushbuttonChessboardCameraCalibration.CData = image;
	
	function onButtonMultiRosbag(src, event)
		ros_data_multiBag_image_point();
	end % onButtonMultiRosbag

  function onButtonChessboardCameraLidarCalibration(src, event)
    configDiag(current_file_path);
	end % onButtonChessboardCameraLidarCalibration

  function onButtonChessboardCameraCalibration(src, event)
    chessboard_camera_calibration(current_file_path);
  end % onButtonChessboardCameraCalibration
  
  function configDiag(current_file_path)
    data_root = current_file_path;
    config_file = 'camera_lidar_calibration_config.yml';

    % create a dialog that get the images-points pairs
    d = dialog('Position',[700 400 600 150],'Name','Get Data root directory and config file');

    % add controls to load the images
    txtDataRootFolder     = uicontrol('Parent',d, ...
                                    'Style','text', ...
                                    'Units', 'normalized', ...
                                    'Position',[0.0 0.85 - 0.05 0.6 0.2], ...
                                    'HorizontalAlignment', 'left', ...
                                    'String','Data Root Folder:');

    editBoxDataRootFolder = uicontrol('Parent', d, ...
                                    'style', 'edit', ...
                                    'Units', 'normalized', ...
                                    'Position', [0.0 0.65 0.8 0.2], ...
                                    'HorizontalAlignment', 'left', ...
                                    'String', current_file_path);
    
    buttonBrowserDataRootFolder = uicontrol('Parent', d, ...
                                        'style', 'pushbutton', ...
                                        'Units', 'normalized', ...
                                        'Position', [0.8 0.65 0.2 0.2], ...
                                        'String', 'browser', ...
                                        'callback', @onBrowserDataRootFolder);

    % add control to load the points
    txtConfigFile     = uicontrol('Parent',d, ...
                                        'Style','text', ...
                                        'Units', 'normalized', ...
                                        'Position',[0.0 0.45 - 0.05 0.6 0.2], ...
                                        'HorizontalAlignment', 'left', ...
                                        'String','Config File:');
    
    editBoxConfigFile = uicontrol('Parent', d, ...
                                    'style', 'edit', ...
                                    'Units', 'normalized', ...
                                    'Position', [0.0 0.25 0.8 0.2], ...
                                    'HorizontalAlignment', 'left', ...
                                    'String', config_file);
    
    buttonBrowserConfigFile = uicontrol('Parent', d, ...
                                        'style', 'pushbutton', ...
                                        'Units', 'normalized', ...
                                        'Position', [0.8 0.25 0.2 0.2], ...
                                        'String', 'browser', ...
                                        'callback', @onBrowserConfigFile);

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

    function onBrowserDataRootFolder(src, event)
        % get the dir to load the images
        folder_name = uigetdir(current_file_path, 'Select the data root directory');
        if isequal(folder_name, 0)
            disp('user selected cancel');
            return;
        else
            disp(['user selected ', folder_name]);
            data_root = folder_name;
            % update the edit box
            set(editBoxDataRootFolder, 'String', data_root);

            % update the config file
            config_file = [data_root '/' 'camera_lidar_calibration_config.yml'];
            set(editBoxConfigFile, 'String', config_file);
        end
    end % onBrowserDataRootFolder

    function onBrowserConfigFile(src, event)
      [FileName, PathName] = uigetfile('*.yml', 'load the config file', data_root);
      if isequal(FileName, 0)
          disp('user selected cancel');
          return;
      else
          disp(['user selected ', [PathName FileName]]);
      end
      config_file = [PathName FileName];
      set(editBoxConfigFile, 'String', config_file);
    end % onBrowserConfigFile

    function onOk(src, event)
      % delete the figure
      delete(gcf);

      chessboard_camera_lidar_calibration(current_file_path, data_root, config_file);
    end % onOk

  end

end