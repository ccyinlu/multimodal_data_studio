function gui = createMultiRosbagTableFigure(app)
    gui = struct();

    % Open a window and add some menus
    gui.handle = figure( ...
        'Name', 'Rosbag Table', ...
        'NumberTitle', 'off', ...
        'MenuBar', 'none', ...
        'Toolbar', 'none', ...
        'HandleVisibility', 'off', ...
        'Visible', 'off');

    gui.handle.SizeChangedFcn = @onSizeChanged;

    gui.columnWidthDefault = [110 80 30 30];
    gui.figureWidthDefault = 255;

    % create an table to this figure
    gui.mTableTopics = uitable(...
                                    'Parent', gui.handle);
    gui.mTableTopics.Data = {};
    gui.mTableTopics.Units = 'Normalized';
    gui.mTableTopics.Position = [0 0 1 1];
    gui.mTableTopics.ColumnName = {...
                                'RosbagFileName', ...
                                'AvailableTopics', ...
                                'NumTopics', ...
                                'Delete'};
    gui.mTableTopics.ColumnFormat = {[], [], [], []};
    gui.mTableTopics.ColumnEditable = [false true false true];
    gui.mTableTopics.ColumnWidth = {...
                                    gui.columnWidthDefault(1) ...
                                    gui.columnWidthDefault(2) ...
                                    gui.columnWidthDefault(3) ...
                                    gui.columnWidthDefault(4)};

    gui.mTableTopics.CellSelectionCallback = @onSelect;

    gui.mTableTopics.CellEditCallback = @onValueChanged;

    function onSizeChanged(src, callbackdata)
        gui.mTableTopics.Units = 'pixels';
        currentFigureWidth = gui.mTableTopics.Position(3);
        gui.mTableTopics.Units = 'Normalized';
        ratio = currentFigureWidth / gui.figureWidthDefault;
        gui.mTableTopics.ColumnWidth = {...
                                        ratio * gui.columnWidthDefault(1) ...
                                        ratio * gui.columnWidthDefault(2) ...
                                        ratio * gui.columnWidthDefault(3) ...
                                        ratio * gui.columnWidthDefault(4)};
    end % onSizeChanged

    function onSelect(src, evt)
        % first judge that the table has content
        if size(evt.Indices, 1)
            if evt.Indices(2) == 1
                % you have select the available topics
                rosbagIndex = evt.Indices(1);
                src.ColumnFormat{2} = app.Rosbag.availableTopics{rosbagIndex};
            end
        end
    end % onSelect

    function onValueChanged(src, evt)
        % first judge that the table has content
        if size(evt.Indices, 1)
            if evt.Indices(2) == 4
                % the delete value changed
                rosbagIndex = evt.Indices(1);
                % you have select the available topics
                app.Rosbag.enable{rosbagIndex} = evt.NewData;
            end
        end
    end % onValueChanged
end