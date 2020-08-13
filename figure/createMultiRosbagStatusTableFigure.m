function gui = createMultiRosbagStatusTableFigure(app)
    gui = struct();

    % Open a window and add some menus
    gui.handle = figure( ...
        'Name', 'Status Table', ...
        'NumberTitle', 'off', ...
        'MenuBar', 'none', ...
        'Toolbar', 'none', ...
        'HandleVisibility', 'off', ...
        'Visible', 'off');

    gui.handle.SizeChangedFcn = @onSizeChanged;

    gui.columnWidthDefault = [122 30];
    gui.figureWidthDefault = 200;

    % create an image table to this figure
    gui.mTableImageTopics = uitable(...
                                    'Parent', gui.handle, ...
                                    'Data', {}, ...
                                    'Units', 'Normalized', ...
                                    'Position', [0 0.9 1 0.1]);
    gui.mTableImageTopics.ColumnName = {...
                                        'AvailableTopics', ...
                                        'bagId'};

    gui.mTableImageTopics.RowName = {...
                                    'image'};
    gui.mTableImageTopics.ColumnFormat = {[], []};
    gui.mTableImageTopics.ColumnEditable = [true false];
    gui.mTableImageTopics.ColumnWidth = {...
                                        gui.columnWidthDefault(1) ...
                                        gui.columnWidthDefault(2)};

    gui.mTableImageTopics.CellEditCallback = @onEditImage;

    % create a point table to this figure
    gui.mTablePointTopics = uitable(...
                                    'Parent', gui.handle, ...
                                    'Data', {}, ...
                                    'Units', 'Normalized', ...
                                    'Position', [0 0.8 1 0.1]);
    gui.mTablePointTopics.ColumnName = {...
                                        'AvailableTopics', ...
                                        'bagId'};

    gui.mTablePointTopics.RowName = {...
                                    'point'};
    gui.mTablePointTopics.ColumnFormat = {[], []};
    gui.mTablePointTopics.ColumnEditable = [true false];
    gui.mTablePointTopics.ColumnWidth = {...
                                        gui.columnWidthDefault(1) ...
                                        gui.columnWidthDefault(2)};

    gui.mTablePointTopics.CellEditCallback = @onEditPoint;

    % create a table to specify the topics to be saved
    gui.mTableSavedTopics = uitable(...
                                    'Parent', gui.handle, ...
                                    'Data', {}, ...
                                    'Units', 'Normalized', ...
                                    'Position', [0 0.0 1 0.8]);
    gui.mTableSavedTopics.ColumnName = {...
                                        'AvailableTopics', ...
                                        'bagId', ...
                                        'S'};

    gui.mTableSavedTopics.ColumnFormat = {[], []};
    gui.mTableSavedTopics.ColumnEditable = [false false true];
    gui.mTableSavedTopics.ColumnWidth = {...
                                        122 ...
                                        30 ...
                                        30};

    gui.mTableSavedTopics.CellEditCallback = @onEditSaveTopics;

    function onSizeChanged(src, callbackdata)
        gui.mTableImageTopics.Units = 'pixels';
        currentFigureWidth = gui.mTableImageTopics.Position(3);
        gui.mTableImageTopics.Units = 'Normalized';
        ratio = currentFigureWidth / gui.figureWidthDefault;
        gui.mTableImageTopics.ColumnWidth = {...
                                            ratio * gui.columnWidthDefault(1) ...
                                            ratio * gui.columnWidthDefault(2)};

        gui.mTablePointTopics.Units = 'pixels';
        currentFigureWidth = gui.mTablePointTopics.Position(3);
        gui.mTablePointTopics.Units = 'Normalized';
        ratio = currentFigureWidth / gui.figureWidthDefault;
        gui.mTablePointTopics.ColumnWidth = {...
                                            ratio * gui.columnWidthDefault(1) ...
                                            ratio * gui.columnWidthDefault(2)};

        gui.mTableSavedTopics.Units = 'pixels';
        currentFigureWidth = gui.mTableSavedTopics.Position(3);
        gui.mTableSavedTopics.Units = 'Normalized';
        ratio = currentFigureWidth / gui.figureWidthDefault;
        gui.mTableSavedTopics.ColumnWidth = {...
                                            ratio * 122 ...
                                            ratio * 30 ...
                                            ratio * 30};
    end % onSizeChanged

    function onEditImage(src, evt)
        if evt.Indices(2) == 1
            % get the belongto according to the new data
            inx = find(ismember(app.RosbagAvailableImageTopics.topics, evt.EditData));
            app.updateAvailableImageTopicsView(inx);
            app.updateCurrentImageTopic(inx);
            % update the play figure info
            app.updatePlayInfo();
            % update the play figure view
            app.updatePlayView();

            % update image and point
            app.updateImagePoint();
        end
    end % onEditImage

    function onEditPoint(src, evt)
        if evt.Indices(2) == 1
            % get the belongto according to the new data
            inx = find(ismember(app.RosbagAvailablePointTopics.topics, evt.EditData));
            app.updateAvailablePointTopicsView(inx);
            app.updateCurrentPointTopic(inx);
            % update the play figure info
            app.updatePlayInfo();
            % update the play figure view
            app.updatePlayView();
            % update image and point
            app.updateImagePoint();
        end
    end % onEditPoint

    function onEditSaveTopics(src, evt)
    end % onEditSaveTopics

end