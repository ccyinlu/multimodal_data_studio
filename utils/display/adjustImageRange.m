function [xdata1, xdata2, ydata1, ydata2] = adjustImageRange(widthHeightRatio, width, height, Limits)
%ADJUSTIMAGERANGE Summary of this function goes here
%   Detailed explanation goes here
    XLimits1 = Limits(1);
    XLimits2 = Limits(2);
    YLimits1 = Limits(3);
    YLimits2 = Limits(4);
    if width/height > widthHeightRatio
        % fit the width
        xdata1 = XLimits1;
        xdata2 = XLimits2;
        ydata1 = 0.5 - (widthHeightRatio/(width/height))/2;
        ydata2 = 0.5 + (widthHeightRatio/(width/height))/2;
    else
        % fit the height
        xdata1 = 0.5 - ((width/height)/widthHeightRatio)/2;
        xdata2 = 0.5 + ((width/height)/widthHeightRatio)/2;
        ydata1 = YLimits1;
        ydata2 = YLimits2;
    end
end

