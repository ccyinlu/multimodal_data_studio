function gui = createMultiRosbagPlayFigure(app)
    gui = struct();

    % Open a window and add some menus
    gui.handle = figure( ...
        'Name', 'Play', ...
        'NumberTitle', 'off', ...
        'MenuBar', 'none', ...
        'Toolbar', 'none', ...
        'HandleVisibility', 'off', ...
        'Visible', 'off');
    
    BoxWidth = 0.13;
    BoxHeight = 0.3;
    BoxLeftMargin = 0.02;
    BoxHMargin = 0.02;
    BoxVPos = 0.3;

    gui.handle.SizeChangedFcn = @onSizeChanged;

    gui.playStatus = false;

    gui.handle.KeyPressFcn = @onKeyPressedFcn;

    % create the edit box for startTime Setting
    gui.mEditBoxStartTimeLocation = [BoxLeftMargin BoxVPos BoxWidth BoxHeight];
    gui.mEditBoxStartTimeLabelLocation = [BoxLeftMargin BoxVPos - BoxHeight BoxWidth BoxHeight];
    gui.mEditBoxStartTime = uicontrol('Parent', gui.handle, 'style', 'edit');
    set(gui.mEditBoxStartTime,'Units', 'normalized');
    set(gui.mEditBoxStartTime, 'Position', gui.mEditBoxStartTimeLocation);
    set(gui.mEditBoxStartTime, 'HorizontalAlignment', 'center');
    set(gui.mEditBoxStartTime, 'FontSize', 10);
    set(gui.mEditBoxStartTime,'String', sprintf('%.3f', app.RosbagPlaySliderStartTime));
    % set the callback for the editbox of startTime
    set(gui.mEditBoxStartTime, 'callback', @onEditStartTime);

    gui.mEditBoxStartTimeLabel = uicontrol('Parent', gui.handle, 'style', 'text');
    set(gui.mEditBoxStartTimeLabel,'Units', 'normalized');
    set(gui.mEditBoxStartTimeLabel, 'Position', gui.mEditBoxStartTimeLabelLocation);
    set(gui.mEditBoxStartTimeLabel, 'HorizontalAlignment', 'center');
    set(gui.mEditBoxStartTimeLabel, 'FontSize', 10);
    set(gui.mEditBoxStartTimeLabel,'String', 'start time');

    % create the edit box for current Setting
    gui.mEditBoxCurrentTimeLocation = [BoxLeftMargin + BoxHMargin + BoxWidth BoxVPos BoxWidth BoxHeight];
    gui.mEditBoxCurrentTimeLabelLocation = [BoxLeftMargin + BoxHMargin + BoxWidth BoxVPos - BoxHeight BoxWidth BoxHeight];
    gui.mEditBoxCurrentTime = uicontrol('Parent', gui.handle, 'style', 'edit');
    set(gui.mEditBoxCurrentTime,'Units', 'normalized');
    set(gui.mEditBoxCurrentTime, 'Position', gui.mEditBoxCurrentTimeLocation);
    set(gui.mEditBoxCurrentTime, 'HorizontalAlignment', 'center');
    set(gui.mEditBoxCurrentTime, 'FontSize', 10);
    set(gui.mEditBoxCurrentTime,'String', sprintf('%.3f', app.RosbagPlaySliderCurrentTime));
    % set the callback for the editbox of startTime
    set(gui.mEditBoxCurrentTime, 'callback', @onEditCurrentTime);

    gui.mEditBoxCurrentTimeLabel = uicontrol('Parent', gui.handle, 'style', 'text');
    set(gui.mEditBoxCurrentTimeLabel,'Units', 'normalized');
    set(gui.mEditBoxCurrentTimeLabel, 'Position', gui.mEditBoxCurrentTimeLabelLocation);
    set(gui.mEditBoxCurrentTimeLabel, 'HorizontalAlignment', 'center');
    set(gui.mEditBoxCurrentTimeLabel, 'FontSize', 10);
    set(gui.mEditBoxCurrentTimeLabel,'String', 'current time');

    % create the edit box for endTime Setting
    gui.mEditBoxEndTimeLocation = [BoxLeftMargin + 2*(BoxHMargin + BoxWidth) BoxVPos BoxWidth BoxHeight];
    gui.mEditBoxEndTimeLabelLocation = [BoxLeftMargin + 2*(BoxHMargin + BoxWidth) BoxVPos - BoxHeight BoxWidth BoxHeight];
    gui.mEditBoxEndTime = uicontrol('Parent', gui.handle, 'style', 'edit');
    set(gui.mEditBoxEndTime,'Units', 'normalized');
    set(gui.mEditBoxEndTime, 'Position', gui.mEditBoxEndTimeLocation);
    set(gui.mEditBoxEndTime, 'HorizontalAlignment', 'center');
    set(gui.mEditBoxEndTime, 'FontSize', 10);
    set(gui.mEditBoxEndTime,'String', sprintf('%.3f', app.RosbagPlaySliderEndTime));
    % set the callback for the editbox of endTime
    set(gui.mEditBoxEndTime, 'callback', @onEditEndTime);

    gui.mEditBoxEndTimeLabel = uicontrol('Parent', gui.handle, 'style', 'text');
    set(gui.mEditBoxEndTimeLabel,'Units', 'normalized');
    set(gui.mEditBoxEndTimeLabel, 'Position', gui.mEditBoxEndTimeLabelLocation);
    set(gui.mEditBoxEndTimeLabel, 'HorizontalAlignment', 'center');
    set(gui.mEditBoxEndTimeLabel, 'FontSize', 10);
    set(gui.mEditBoxEndTimeLabel,'String', 'end time');

    % create the text box for maxTime Setting
    gui.mTextBoxMaxTimeLocation = [BoxLeftMargin + 3*(BoxHMargin + BoxWidth) BoxVPos - 0.05 BoxWidth BoxHeight];
    gui.mTextBoxMaxTimeLabelLocation = [BoxLeftMargin + 3*(BoxHMargin + BoxWidth) BoxVPos - BoxHeight BoxWidth BoxHeight];
    gui.mTextBoxMaxTime = uicontrol('Parent', gui.handle, 'style', 'text');
    set(gui.mTextBoxMaxTime,'Units', 'normalized');
    set(gui.mTextBoxMaxTime, 'Position', gui.mTextBoxMaxTimeLocation);
    set(gui.mTextBoxMaxTime, 'HorizontalAlignment', 'center');
    set(gui.mTextBoxMaxTime, 'FontSize', 10);
    set(gui.mTextBoxMaxTime,'String', sprintf('%.3f', app.RosbagPlaySliderMaxTime));

    gui.mTextBoxEndTimeLabel = uicontrol('Parent', gui.handle, 'style', 'text');
    set(gui.mTextBoxEndTimeLabel,'Units', 'normalized');
    set(gui.mTextBoxEndTimeLabel, 'Position', gui.mTextBoxMaxTimeLabelLocation);
    set(gui.mTextBoxEndTimeLabel, 'HorizontalAlignment', 'center');
    set(gui.mTextBoxEndTimeLabel, 'FontSize', 10);
    set(gui.mTextBoxEndTimeLabel,'String', 'max time');

    % create the slider control for the playing
    gui.mSliderBoxPlayLocation = [0 BoxVPos + BoxHeight + 0.1 1 0.2];
    gui.mSliderBoxPlay = uicontrol('Parent', gui.handle, 'style', 'slider');
    set(gui.mSliderBoxPlay,'Units', 'normalized');
    set(gui.mSliderBoxPlay,'Min', 1);
    set(gui.mSliderBoxPlay,'Max', 10);
    set(gui.mSliderBoxPlay,'Value', 1);
    set(gui.mSliderBoxPlay,'SliderStep', [0.01 0.01]);
    set(gui.mSliderBoxPlay, 'Position', gui.mSliderBoxPlayLocation);
    set(gui.mSliderBoxPlay, 'HorizontalAlignment', 'center');
    % set the callback for the slider
    set(gui.mSliderBoxPlay, 'callback', @onSliderValueChanged);

    % create the pushbutton previous2 control for playing
    gui.mPushbuttonBoxPrevious2Location = [BoxLeftMargin + 4*(BoxHMargin + BoxWidth) BoxVPos - 0.05 BoxWidth/4 BoxHeight];
    gui.mPushbuttonBoxPrevious2 = uicontrol('Parent', gui.handle, 'style', 'pushbutton', 'Units', 'normalized', 'Position', gui.mPushbuttonBoxPrevious2Location);
    jh = findjobj(gui.mPushbuttonBoxPrevious2); 
    jh.setFlyOverAppearance(true); % set the button boarder invisible

    % create the pushbutton previous control for playing
    gui.mPushbuttonBoxPreviousLocation = [BoxLeftMargin + 4*(BoxHMargin + BoxWidth) + (BoxWidth/4 + 0.01) BoxVPos - 0.05 BoxWidth/4 BoxHeight];
    gui.mPushbuttonBoxPrevious = uicontrol('Parent', gui.handle, 'style', 'pushbutton', 'Units', 'normalized', 'Position', gui.mPushbuttonBoxPreviousLocation);
    jh = findjobj(gui.mPushbuttonBoxPrevious);
    jh.setFlyOverAppearance(true); % set the button boarder invisible
    set(gui.mPushbuttonBoxPrevious, 'callback', @onPlayPreviousButtonCallback);

    % create the pushbutton play control for playing
    gui.mPushbuttonBoxPlayLocation = [BoxLeftMargin + 4*(BoxHMargin + BoxWidth) + 2*(BoxWidth/4 + 0.01) BoxVPos - 0.05 BoxWidth/4 BoxHeight];
    gui.mPushbuttonBoxPlay = uicontrol('Parent', gui.handle, 'style', 'pushbutton', 'Units', 'normalized', 'Position', gui.mPushbuttonBoxPlayLocation);
    jh = findjobj(gui.mPushbuttonBoxPlay);
    jh.setFlyOverAppearance(true); % set the button boarder invisible
    set(gui.mPushbuttonBoxPlay, 'callback', @onPlayButtonCallback);


    % create the pushbutton next control for playing
    gui.mPushbuttonBoxNextLocation = [BoxLeftMargin + 4*(BoxHMargin + BoxWidth) + 3*(BoxWidth/4 + 0.01) BoxVPos - 0.05 BoxWidth/4 BoxHeight];
    gui.mPushbuttonBoxNext = uicontrol('Parent', gui.handle, 'style', 'pushbutton', 'Units', 'normalized', 'Position', gui.mPushbuttonBoxNextLocation);
    jh = findjobj(gui.mPushbuttonBoxNext);
    jh.setFlyOverAppearance(true); % set the button boarder invisible
    set(gui.mPushbuttonBoxNext, 'callback', @onPlayNextButtonCallback);

    % create the pushbutton next2 control for playing
    gui.mPushbuttonBoxNext2Location = [BoxLeftMargin + 4*(BoxHMargin + BoxWidth) + 4*(BoxWidth/4 + 0.01) BoxVPos - 0.05 BoxWidth/4 BoxHeight];
    gui.mPushbuttonBoxNext2 = uicontrol('Parent', gui.handle, 'style', 'pushbutton', 'Units', 'normalized', 'Position', gui.mPushbuttonBoxNext2Location);
    jh = findjobj(gui.mPushbuttonBoxNext2);
    jh.setFlyOverAppearance(true); % set the button boarder invisible

    % create the pushbutton sync status for playing
    gui.mPushbuttonBoxSyncStatusLocation = [BoxLeftMargin + 4*(BoxHMargin + BoxWidth) + 5*(BoxWidth/4 + 0.01) BoxVPos - 0.05 BoxWidth/4 - 0.005 BoxHeight];
    gui.mPushbuttonBoxSyncStatus = uicontrol('Parent', gui.handle, 'style', 'pushbutton', 'Units', 'normalized', 'Position', gui.mPushbuttonBoxSyncStatusLocation);
    jh = findjobj(gui.mPushbuttonBoxSyncStatus);
    jh.setFlyOverAppearance(true); % set the button boarder invisible

    drawButtonImage();
    drawSyncStatusButtonImage();

    function onSizeChanged(src, callbackdata)
        %image and button list
        drawButtonImage();
        drawSyncStatusButtonImage();
    end % onSizeChanged

    function onEditStartTime(src, event)
        % change the start time
        startTime = str2num(src.String);
        if startTime >= 0.0 & startTime <= app.RosbagPlaySliderEndTime
            app.RosbagPlaySliderStartTime = startTime;
            % change the current time according to the startTime
            if app.RosbagPlaySliderCurrentTime < app.RosbagPlaySliderStartTime
                app.RosbagPlaySliderCurrentTime = app.RosbagPlaySliderStartTime;
                app.updateImagePoint();
                set(gui.mEditBoxCurrentTime,'String', sprintf('%.3f', app.RosbagPlaySliderCurrentTime));
                set(app.guiPlay.mSliderBoxPlay,'Value', app.RosbagPlaySliderCurrentTime);
            end
        end
        set(gui.mEditBoxStartTime,'String', sprintf('%.3f', app.RosbagPlaySliderStartTime));
    end % onEditStartTime

    function onEditCurrentTime(src, event)
        % change the current time
        currentTime = str2num(src.String);
        if currentTime >= app.RosbagPlaySliderStartTime & currentTime <= app.RosbagPlaySliderEndTime
            app.RosbagPlaySliderCurrentTime = currentTime;
            app.updateImagePoint();
            set(gui.mEditBoxCurrentTime,'String', sprintf('%.3f', app.RosbagPlaySliderCurrentTime));
            set(app.guiPlay.mSliderBoxPlay,'Value', app.RosbagPlaySliderCurrentTime);

        end
        set(gui.mEditBoxStartTime,'String', sprintf('%.3f', app.RosbagPlaySliderStartTime));
    end % onEditCurrentTime

    function onEditEndTime(src, event)
        % change the end time
        endTime = str2num(src.String);
        if endTime >= app.RosbagPlaySliderStartTime & endTime <= app.RosbagPlaySliderMaxTime
            app.RosbagPlaySliderEndTime = endTime;
            % change the current time according to the endTime
            if app.RosbagPlaySliderCurrentTime > app.RosbagPlaySliderEndTime
                app.RosbagPlaySliderCurrentTime = app.RosbagPlaySliderEndTime;
                app.updateImagePoint();
                set(gui.mEditBoxCurrentTime,'String', sprintf('%.3f', app.RosbagPlaySliderCurrentTime));
                set(app.guiPlay.mSliderBoxPlay,'Value', app.RosbagPlaySliderCurrentTime);
            end
        end
        set(gui.mEditBoxEndTime,'String', sprintf('%.3f', app.RosbagPlaySliderEndTime));
    end % onEditEndTime

    function onSliderValueChanged(src, event)
        if src.Value < app.RosbagPlaySliderStartTime
            src.Value = app.RosbagPlaySliderStartTime;
        end
        if src.Value > app.RosbagPlaySliderEndTime
            src.Value = app.RosbagPlaySliderEndTime;
        end
        app.RosbagPlaySliderCurrentTime = src.Value;
        app.updateImagePoint();
        set(gui.mEditBoxCurrentTime,'String', sprintf('%.3f', app.RosbagPlaySliderCurrentTime));
    end % onSliderValueChanged

    function onPlayButtonCallback(src, event)
        if gui.playStatus
            gui.playStatus = false;
            % update the button image
            drawButtonImage();
        else
            % play the image from the current start time to the end time
            gui.playStatus = true;
            % update the button image
            drawButtonImage();
            while (app.RosbagPlaySliderCurrentTime + app.RosbagPlayStep < app.RosbagPlaySliderEndTime) & gui.playStatus
                app.RosbagPlaySliderCurrentTime = app.RosbagPlaySliderCurrentTime + app.RosbagPlayStep;
                app.updateImagePoint();
                % update the slider current time
                set(gui.mEditBoxCurrentTime,'String', sprintf('%.3f', app.RosbagPlaySliderCurrentTime));
                % update the slider
                app.updatePlayView();
            end
            % judge if the play was paused
            if gui.playStatus
                % the whole play was finished, and replay from the start
                app.RosbagPlaySliderCurrentTime = app.RosbagPlaySliderStartTime;
                set(gui.mEditBoxCurrentTime,'String', sprintf('%.3f', app.RosbagPlaySliderCurrentTime));
                % reset the button icon
                gui.playStatus = false;
                % update the button icon
                drawButtonImage();
                % reset the slider value
                app.updatePlayView();
            end
        end
    end % onPlayButtonCallback

    function onPlayNextButtonCallback(src, event)
        app.RosbagPlaySliderCurrentTime = app.RosbagPlaySliderCurrentTime + app.RosbagPlayStep;
        if app.RosbagPlaySliderCurrentTime < app.RosbagPlaySliderStartTime
            app.RosbagPlaySliderCurrentTime = app.RosbagPlaySliderStartTime;
        end
        if app.RosbagPlaySliderCurrentTime > app.RosbagPlaySliderEndTime
            app.RosbagPlaySliderCurrentTime = app.RosbagPlaySliderEndTime;
        end
        app.updateImagePoint();
        set(gui.mEditBoxCurrentTime,'String', sprintf('%.3f', app.RosbagPlaySliderCurrentTime));
        set(app.guiPlay.mSliderBoxPlay,'Value', app.RosbagPlaySliderCurrentTime);
    end % onPlayNextButtonCallback

    function onPlayPreviousButtonCallback(src, event)
        app.RosbagPlaySliderCurrentTime = app.RosbagPlaySliderCurrentTime - app.RosbagPlayStep;
        if app.RosbagPlaySliderCurrentTime < app.RosbagPlaySliderStartTime
            app.RosbagPlaySliderCurrentTime = app.RosbagPlaySliderStartTime;
        end
        if app.RosbagPlaySliderCurrentTime > app.RosbagPlaySliderEndTime
            app.RosbagPlaySliderCurrentTime = app.RosbagPlaySliderEndTime;
        end
        app.updateImagePoint();
        set(gui.mEditBoxCurrentTime,'String', sprintf('%.3f', app.RosbagPlaySliderCurrentTime));
        set(gui.mSliderBoxPlay,'Value', app.RosbagPlaySliderCurrentTime);
    end % onPlayPreviousButtonCallback

    function drawSyncStatusButtonImage()
        images={...
                fullfile(app.current_dir, 'icons/ok.png'), ...
                fullfile(app.current_dir, 'icons/fail.png')};
        buttons={...
                gui.mPushbuttonBoxSyncStatus};

        bgColor = [0.94 0.94 0.94];

        % redraw the syncStatus
        if app.current_syncStatus
            image = imread(images{1}, 'BackGroundColor', bgColor);
        else
            image = imread(images{2}, 'BackGroundColor', bgColor);
        end
        buttons{1}.Units='pixels';
        image = imresize(image, fliplr(buttons{1}.Position(1,3:4)));
        buttons{1}.Units='normalized';
        buttons{1}.CData=image;
    end

    function drawButtonImage()
        images={...
                fullfile(app.current_dir, 'icons/previous2.png'), ...
                fullfile(app.current_dir, 'icons/previous.png'), ...
                fullfile(app.current_dir, 'icons/play.png'), ...
                fullfile(app.current_dir, 'icons/pause.png'), ...
                fullfile(app.current_dir, 'icons/next.png'), ...
                fullfile(app.current_dir, 'icons/next2.png')};
        buttons={...
                 gui.mPushbuttonBoxPrevious2, ...
                 gui.mPushbuttonBoxPrevious, ...
                 gui.mPushbuttonBoxPlay, ...
                 gui.mPushbuttonBoxNext, ...
                 gui.mPushbuttonBoxNext2, ...
                 gui.mPushbuttonBoxSyncStatus};

        bgColor = [0.94 0.94 0.94];
        
        % redraw the previous2
        image = imread(images{1}, 'BackGroundColor', bgColor);
        buttons{1}.Units='pixels';
        image = imresize(image, 0.7 * fliplr(buttons{1}.Position(1,3:4))); % resize the png to 0.8 of the button boarder
        buttons{1}.Units='normalized';
        buttons{1}.CData=image;

        % redraw the previous
        image = imread(images{2}, 'BackGroundColor', bgColor);
        buttons{2}.Units='pixels';
        image = imresize(image, fliplr(buttons{2}.Position(1,3:4)));
        buttons{2}.Units='normalized';
        buttons{2}.CData=image;

        % redraw the play
        if gui.playStatus
            image = imread(images{4}, 'BackGroundColor', bgColor);
        else
            image = imread(images{3}, 'BackGroundColor', bgColor);
        end
        buttons{3}.Units='pixels';
        image = imresize(image, fliplr(buttons{3}.Position(1,3:4)));
        buttons{3}.Units='normalized';
        buttons{3}.CData=image;
        
        % redraw the next
        image = imread(images{5}, 'BackGroundColor', bgColor);
        buttons{4}.Units='pixels';
        image = imresize(image, fliplr(buttons{4}.Position(1,3:4)));
        buttons{4}.Units='normalized';
        buttons{4}.CData=image;

        % redraw the next2
        image = imread(images{6}, 'BackGroundColor', bgColor);
        buttons{5}.Units='pixels';
        image = imresize(image, 0.7 * fliplr(buttons{5}.Position(1,3:4))); % resize the png to 0.8 of the button boarder
        buttons{5}.Units='normalized';
        buttons{5}.CData=image;
    end

    function onKeyPressedFcn(src, event)
        % key press fcn callback
        switch (event.Key)
        case 'rightarrow'
            onPlayNextButtonCallback(gui.mPushbuttonBoxNext, []);
        case 'leftarrow'
            onPlayPreviousButtonCallback(gui.mPushbuttonBoxNext, []);
        end
    end % onKeyPressedFcn
end