function se3 = euler2se3(euler)
    if size(euler, 1) == 6 && size(euler, 2) == 1
        eulerRefine = euler;
    elseif size(euler, 1) == 1 && size(euler, 2) == 6
        eulerRefine = euler';
    else
        error('input euler must be 1x6 or 6x1, while the current: %dx%d', size(euler, 1), size(euler, 2));
    end

    se3 = euler2se3Mex(eulerRefine);
end