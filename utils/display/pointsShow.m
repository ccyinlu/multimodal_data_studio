function pointsShow(ax, points, limits, Ilimits, showType, pointSize)
    % show the 3D points according to the showType
    mshowType = 0;

    %% set the colormap
    CC = jet(256);

    %% set the default intensity min and max
    if isempty(Ilimits)
        Ilimits = [0 255];
    end

    %% crop the points according to the limits
    indexValid = points(:, 1) > limits(1) & points(:, 1) < limits(2) & points(:, 2) > limits(3) & points(:, 2) < limits(4) & points(:, 3) > limits(5) & points(:, 3) < limits(6);
    points = points(indexValid, :);

    if nargin < 3
        disp('showtype not specified, default to PLAIN');
        disp('you can specify the showType to 1: plain, 2: height, 3: range, 4: intensity');
        mshowType = 1;
    else
        if strcmp(showType, 'plain')
            mshowType = 1;
        elseif strcmp(showType, 'height')
            mshowType = 2;
        elseif strcmp(showType, 'range')
            mshowType = 3;
        elseif strcmp(showType, 'intensity')
            mshowType = 4;
        else
            disp('showType error, default to PLAIN');
            disp('you can specify the showType to 1: plain, 2: height, 3: range, 4: intensity');
            mshowType = 1;
    end
    
    if (size(points, 2) < 4) && mshowType == 4
        error('showType specified to intensity but no intensity provided');
    end

    if mshowType == 1
        %% plain color, set the color to 'blue'
        C = 'w';
    end

    if mshowType == 2
        %% set the color according to the height
        C = CC(floor((points(:, 3) - limits(5))/(limits(6) - limits(5)) * 255 + 1), :);
    end

    if mshowType == 3
        %% set the color according to the range
        xRange = max(abs(limits(1)), abs(limits(2)));
        yRange = max(abs(limits(3)), abs(limits(4)));
        zRange = max(abs(limits(5)), abs(limits(6)));
        r = sqrt(points(:, 1).^2 + points(:, 2).^2 + points(:, 3).^2);
        r_max = sqrt(xRange.^2 + yRange.^2 + zRange.^2);
        C = CC(floor(r/r_max * 255 + 1), :);
    end

    if mshowType == 4
        intensity = points(:, 4);
        intensityLow = max(min(intensity), Ilimits(1));
        % intensityHigh = max(points(:, 4));
        intensityHigh = min(std(intensity) * 3 + mean(intensity), Ilimits(2));

        intensity(intensity <= intensityLow) = intensityLow; 
        intensity(intensity >= intensityHigh) = intensityHigh; 
        C = CC(floor((intensity - intensityLow)/(intensityHigh - intensityLow) * 255 + 1), :);
    end

    scatter3(ax, points(:, 1), points(:, 2), points(:, 3), pointSize, C, 'filled');
end