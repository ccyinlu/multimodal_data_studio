function mask = generateMaskFromCorners(imagePoints, imageSize, patternSize)

    % imagePoints, Nx2

    % patch the missing corners
    point_top_left = 2*imagePoints(patternSize(1)*(1-1) + 1, :) - imagePoints(patternSize(1)*(2-1) + 2, :);
    point_top_right = 2*imagePoints(patternSize(1)*(1-1) + patternSize(1), :) - imagePoints(patternSize(1)*(2-1) + patternSize(1) - 1, :);
    point_bottom_left = 2*imagePoints(patternSize(1)*(patternSize(2)-1) + 1, :) - imagePoints(patternSize(1)*(patternSize(2) - 1 - 1) + 2, :);
    point_bottom_right = 2*imagePoints(patternSize(1)*(patternSize(2)-1) + patternSize(1), :) - imagePoints(patternSize(1)*(patternSize(2) - 1 - 1) + patternSize(1) - 1, :);
    
    wpConvHull = [  point_top_left; ...
                    point_bottom_left; ...
                    point_bottom_right; ...
                    point_top_right;];

    row = imageSize(1);
    col = imageSize(2);

    [X, Y] = meshgrid(1:col, 1:row);
    index = inpolygon(X(:), Y(:), wpConvHull(:, 1), wpConvHull(:, 2));
    mask = reshape(index, row, col);
end