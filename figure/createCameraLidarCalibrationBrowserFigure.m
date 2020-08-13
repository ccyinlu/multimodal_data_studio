function gui = createCameraLidarCalibrationBrowserFigure(app)
    gui = struct();

    % Open a window and add some menus
    gui.handle = figure( ...
        'Name', 'Browser', ...
        'NumberTitle', 'off', ...
        'MenuBar', 'none', ...
        'ToolBar', 'none', ...
        'HandleVisibility', 'off', ...
        'Visible', 'off');

    gui.handle.SizeChangedFcn = @onSizeChanged;
    gui.mTableImageColumnWidthRatio = [0.5 0.4 0.05];

    % create an image table to this figure
    gui.mTableImage = uitable(...
                            'Parent', gui.handle, ...
                            'Data', {}, ...
                            'Units', 'Normalized', ...
                            'Position', [0 0 1 1]);
    gui.mTableImage.RowName = [];
    gui.mTableImage.ColumnName = [];
    gui.mTableImage.ColumnFormat = {[], 'char', []};
    gui.mTableImage.ColumnEditable = [false false true];

    gui.mTableImage.CellSelectionCallback = @onSelectBrowser;

    gui.mTableImage.CellEditCallback = @onValueChanged;

    function onSizeChanged(~, ~)
        gui.mTableImage.Units = 'pixels';
        figureCurrentWidth = gui.mTableImage.Position(3);
        gui.mTableImage.Units = 'Normalized';
        gui.mTableImage.ColumnWidth = {...
                                        figureCurrentWidth * gui.mTableImageColumnWidthRatio(1) ...
                                        figureCurrentWidth * gui.mTableImageColumnWidthRatio(2) ...
                                        figureCurrentWidth * gui.mTableImageColumnWidthRatio(3)};

        % set the row height
        app.redrawBrowserTableHeight();
    end % onSizeChanged

    function onSelectBrowser(~, data)
        if ~isempty(data) && ~isempty(data.Indices)
            if ~isequal(data.Indices(2), 3)
                % we can respond to both the first column and the second column
                app.current_index = data.Indices(1);
                % update the image and point
                app.updateImagePoint();
                % update the image view
                app.updateImageView();
                % update the image points view
                app.refreshImagePointsTableData();
                % refresh the lidar point chessboard points
                app.updatePointView();
                % refresh the lidar points table view
                app.refreshLidarPointsTableData();
            end
        end
    end % onSelectBrowser

    function onValueChanged(~, data)
        if ~isempty(data) && ~isempty(data.Indices)
            if isequal(data.Indices(2), 3)
                app.cameraLidarextrinsicCalibrationData.enable{data.Indices(1)} = data.NewData;
            end
        end
    end % onValueChanged
end