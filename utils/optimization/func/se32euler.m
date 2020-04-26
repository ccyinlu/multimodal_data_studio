function euler = se32euler(se3)
    if size(se3, 1) == 6 && size(se3, 2) == 1
        se3Refine = se3;
    elseif size(se3, 1) == 1 && size(se3, 2) == 6
        se3Refine = se3';
    else
        error('input se3 must be 1x6 or 6x1, while the current: %dx%d', size(se3, 1), size(se3, 2));
    end
    eulerTransform = se32eulerMex(se3Refine);
    eulerAngle = rotm2eul(eulerTransform(1:3, 1:3));
    eulerVec = eulerTransform(1:3, 4);
    euler = [eulerAngle(1) eulerAngle(2) eulerAngle(3) eulerVec(1) eulerVec(2) eulerVec(3)]';
end