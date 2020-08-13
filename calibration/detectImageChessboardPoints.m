function [imagePointsConsistency, boardSizeConsistency, imageUsedConsistency, imagePoints, boardSize, imageUsed] = detectImageChessboardPoints(imageFilenames, algo, patternSize)
    % detect the chessboard points according to the algos
    imagePointsConsistency = [];
    boardSizeConsistency = [];
    imageUsedConsistency = [];
    imagePoints = {};
    boardSize = {};
    imageUsed = [];
    
    imagesNum = length(imageFilenames);
    %global ifCancel;
    ifCancel = false;
    h = waitbar(0, 'detect the chessboard points', 'CreateCancelBtn',@waitbar_cancel, 'Name', 'analysis images');

    imageUsedConsistency = zeros(imagesNum, 1);
    imageUsed = zeros(imagesNum, 1);

    for i = 1 : imagesNum
        if ~ifCancel
            switch(algo)
            case 'matlab'
                % use the matlab in-built functions to detected the chessboard points
                [imagePoints_, boardSize_, imageUsed_] = detectCheckerboardPoints(imageFilenames{i});
            otherwise
                warningUnknownAlgoDialog();
                return
            end

            imageUsed(i) = imageUsed_;
            imageUsedConsistency(i) = imageUsed_;

            if imageUsed_
                if size(imagePoints_, 1) ==  patternSize
                    imagePointsConsistency = cat(3, imagePointsConsistency, imagePoints_);
                    boardSizeConsistency = boardSize_;
                else
                    imageUsedConsistency(i) = 0;
                end
                % cat the unConsistency result
                imagePoints = cat(1, imagePoints, imagePoints_);
                boardSize = cat(1, boardSize, boardSize_);
            else
                % cat the unConsistency result
                imagePoints = cat(1, imagePoints, 0);
                boardSize = cat(1, boardSize, 0);
            end

            progress = i/imagesNum;
            msgs = sprintf('detecting chessboard points in image %d/%d', i, imagesNum);
            waitbar(progress, h, msgs, 'Name', 'analysis images');
        end
    end
    delete(h);

    imageUsedConsistency = imageUsedConsistency == 1;
    imageUsed = imageUsed == 1;
    % show the results
    totalImages = imagesNum;
    imageCalibImages = sum(imageUsedConsistency);
    totalCalibImages = sum(imageUsed);
    rejectImageCalibImages = totalImages - imageCalibImages;
    rejectTotalCalibImages = totalImages - totalCalibImages;
    resultMsgsBox();

    function waitbar_cancel(src, event)
        %global ifCancel;
        ifCancel = true;
        delete(src);
    end

    function warningUnknownAlgoDialog
        str = sprintf('Specified aglos not supported! \r\nPlease check your configuration');
        h = warndlg(str, 'Invalid Algo');
        uiwait(h);
    end

    function resultMsgsBox
        str1 = sprintf('total images:                               %d', totalImages);
        str2 = sprintf('images for intrinsic calib:            %d', imageCalibImages);
        str3 = sprintf('images for extrinsic calib:           %d', totalCalibImages);
        str4 = sprintf('reject images for intrinsic calib:  %d', rejectImageCalibImages);
        str5 = sprintf('reject images for extrinsic calib: %d', rejectTotalCalibImages);
        uiwait(msgbox({str1 str2 str3 str4 str5}, 'Results'));
    end
end