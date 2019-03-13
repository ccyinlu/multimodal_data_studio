function imagePointPairsStruct = loadImagePointPairs(imagesDir, pointsDir)
    % load the images and points information to the struct
    % support image format
    supportImageFormat = {'.jpg', '.png', '.jpeg', '.tiff'};
    supportPointFormat = {'.pcd', '.bin', '.txt'};

    imagePointPairsStruct = struct();
    imagePointPairsStruct.rawPairNum = 0;
    imagePointPairsStruct.rawImagesFilename = {};
    imagePointPairsStruct.rawPointsFilename = {};

    % first check the existance of the directory
    if exist(imagesDir, 'dir') ~= 7
        warningInvalidImageDirDialog();
        imagePointPairsStruct = [];
        return;
    end

    if exist(pointsDir, 'dir') ~= 7
        warningInvalidPointDirDialog();
        imagePointPairsStruct = [];
        return;
    end

    % search all the images under the imageDir
    AllImageFiles = dir([imagesDir '/*.*']);
    imageFilenames = {};
    imageFilenamesId = {};
    imageFilenamesCount = 0;
    for i = 1 : length(AllImageFiles)
        % ignore the folder
        if ~(AllImageFiles(i).isdir)
            % get the filename extenntion
            [~, id, ext] = fileparts(AllImageFiles(i).name);
            if ismember(ext, supportImageFormat)
                imageFilenamesCount = imageFilenamesCount + 1;
                imageFilenames{imageFilenamesCount} = AllImageFiles(i).name;
                imageFilenamesId{imageFilenamesCount} = id;
            end
        end
    end

    % search all the points under the pointDir
    AllPointFiles = dir([pointsDir '/*.*']);
    pointFilenames = {};
    pointFilenamesId = {};
    pointFilenamesCount = 0;
    for i = 1 : length(AllPointFiles)
        % ignore the folder
        if ~(AllPointFiles(i).isdir)
            % get the filename extenntion
            [~, id, ext] = fileparts(AllPointFiles(i).name);
            if ismember(ext, supportPointFormat)
                pointFilenamesCount = pointFilenamesCount + 1;
                pointFilenames{pointFilenamesCount} = AllPointFiles(i).name;
                pointFilenamesId{pointFilenamesCount} = id;
            end
        end
    end

    % compare the image name and point name, add a pair when the names are the same

    if imageFilenamesCount
        if pointFilenamesCount
            imagePointPairsStruct.rawPairNum = 0;
            for i = 1 : imageFilenamesCount
                % get the index of the point file
                indexPointFilename = find(ismember(pointFilenamesId, imageFilenamesId{i}));
                if ~isempty(indexPointFilename)
                    imagePointPairsStruct.rawPairNum = imagePointPairsStruct.rawPairNum + 1;
                    imagePointPairsStruct.rawImagesFilename = cat(1, imagePointPairsStruct.rawImagesFilename, [imagesDir '/' imageFilenames{i}]);
                    imagePointPairsStruct.rawPointsFilename = cat(1, imagePointPairsStruct.rawPointsFilename, [pointsDir '/' pointFilenames{indexPointFilename}]);
                end
            end

            if imagePointPairsStruct.rawPairNum == 0
                warningNoImagePointPairsFoundDialog();
                imagePointPairsStruct = [];
                return;
            end
        else
            warningNoPointsFoundDialog();
            imagePointPairsStruct = [];
            return;
        end
    else
        warningNoImagesFoundDialog();
        imagePointPairsStruct = [];
        return;
    end

    function warningInvalidImageDirDialog
        str = sprintf('Specified image directory not found! \r\nPlease check your configuration');
        h = warndlg(str, 'Invalid Image Directory');
        uiwait(h);
    end

    function warningInvalidPointDirDialog
        str = sprintf('Specified point directory not found! \r\nPlease check your configuration');
        h = warndlg(str, 'Invalid Point Directory');
        uiwait(h);
    end

    function warningNoImagesFoundDialog
        str = sprintf('No images found under the imagesDir \r\nPlease check your configuration');
        h = warndlg(str, 'No Images Found');
        uiwait(h);
    end

    function warningNoPointsFoundDialog
        str = sprintf('No points found under the pointsDir \r\nPlease check your configuration');
        h = warndlg(str, 'No Points Found');
        uiwait(h);
    end

    function warningNoImagePointPairsFoundDialog
        str = sprintf('No image-point pairs found! \r\nPlease check your configuration');
        h = warndlg(str, 'No Image-Point-Pairs Found');
        uiwait(h);
    end
end