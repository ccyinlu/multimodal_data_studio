function distanceMatFull = distanceMatSegmentation(points, planeModel, distanceMatSegmentationParams)
    % segment the points according to the distance matrix

    horizontal_res = distanceMatSegmentationParams.horizontal_res;

    vertical_theta = distanceMatSegmentationParams.vertical_theta;
    vertical_theta = vertical_theta/180*pi;

    % calc the distance between the points and the plane
    plane_func_params = planeModel.Parameters;
    plane_normal_norm = norm(plane_func_params(1:3));

    points_extend = [points(:, 1:3) ones(size(points, 1), 1)];
    distance = abs(points_extend * plane_func_params') / plane_normal_norm;

    % calc the vertical angle and horizontal angle
    v_angle = atan2(points(:, 3), sqrt(points(:, 1).*points(:, 1) + points(: ,2) .* points(:, 2)));
    % if(isequal(pointType,'hesai_p40p'))
    %     h_angle = atan2(points(: ,1), -points(: ,2));
    % elseif (isequal(pointType,'prescan_p40p'))
    %     h_angle = atan2(points(: ,1), points(: ,2));
    % end

    h_angle = atan2(points(: ,2), points(: ,1));

    h_angle_index = floor((h_angle + pi )/(2*pi)*(2*pi/horizontal_res));
    h_angle_index_min = min(h_angle_index);
    h_angle_index_max = max(h_angle_index);

    h_angle_index = h_angle_index - h_angle_index_min + 1;
    h_angle_num = h_angle_index_max - h_angle_index_min + 1;

    v_angle_index = floor(zeros(length(v_angle), 1));

    for i = 1 : length(v_angle)

        v_index = 1;
        v_angle_res_min = 1000000;
        for j = 1 : length(vertical_theta)
            v_angle_res = abs(v_angle(i) - vertical_theta(j));
            if(v_angle_res < v_angle_res_min)
                v_angle_res_min = v_angle_res;
                v_index = j;
            else
                break;
            end
        end

        v_angle_index(i) = v_index;
    end

    v_angle_num = length(vertical_theta);

    distanceMatFull = NaN(v_angle_num, h_angle_num, 6);

    for i = 1 : size(points, 1)
        cur_row_index = v_angle_index(i);
        cur_col_index = h_angle_index(i);
        cur_x = points(i, 1);
        cur_y = points(i, 2);
        cur_z = points(i, 3);
        cur_d = distance(i);
        cur_intensity = points(i, 4);
        distanceMatFull(cur_row_index, cur_col_index, 1) = cur_x;
        distanceMatFull(cur_row_index, cur_col_index, 2) = cur_y;
        distanceMatFull(cur_row_index, cur_col_index, 3) = cur_z;
        distanceMatFull(cur_row_index, cur_col_index, 4) = cur_d;
        distanceMatFull(cur_row_index, cur_col_index, 5) = i;
        distanceMatFull(cur_row_index, cur_col_index, 6) = cur_intensity;
    end

end