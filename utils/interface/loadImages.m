function imageStruct = loadImages(imagesDir)
    % load the images information to the struct
    % support image format
    supportImageFormat = {'.jpg', '.png', '.jpeg', '.tiff'};

    imageStruct = struct();
    imageStruct.rawImagesFilename = {};

    % first check the existance of the directory
    if exist(imagesDir, 'dir') ~= 7
        warningInvalidImageDirDialog();
        imageStruct = [];
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

    % compare the image name and point name, add a pair when the names are the same

    if imageFilenamesCount
        for i = 1 : imageFilenamesCount
            imageStruct.rawImagesFilename = cat(1, imageStruct.rawImagesFilename, [imagesDir '/' imageFilenames{i}]);
        end
    else
        warningNoImagesFoundDialog();
        imageStruct = [];
        return;
    end

    function warningInvalidImageDirDialog
        str = sprintf('Specified image directory not found! \r\nPlease check your configuration');
        h = warndlg(str, 'Invalid Image Directory');
        uiwait(h);
    end

    function warningNoImagesFoundDialog
        str = sprintf('No images found under the imagesDir \r\nPlease check your configuration');
        h = warndlg(str, 'No Images Found');
        uiwait(h);
    end
end