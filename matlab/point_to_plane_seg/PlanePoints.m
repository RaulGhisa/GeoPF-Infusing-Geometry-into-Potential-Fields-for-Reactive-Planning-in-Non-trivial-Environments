function points = PlanePoints(obs_center, width, plane_direction)
    if plane_direction == "X"
        widths = PlaneWidthX(width);
        width1 = widths(1, :);
        width2 = widths(2, :);
        width3 = widths(3, :);
        width4 = widths(4, :);
    elseif plane_direction == "Y"
        widths = PlaneWidthY(width);
        width1 = widths(1, :);
        width2 = widths(2, :);
        width3 = widths(3, :);
        width4 = widths(4, :);
    elseif plane_direction == "Z"
        widths = PlaneWidthZ(width);
        width1 = widths(1, :);
        width2 = widths(2, :);
        width3 = widths(3, :);
        width4 = widths(4, :);
    end

    obs_point1 = [[obs_center + width1]];
    obs_point2 = [[obs_center + width2]];
    obs_point3 = [[obs_center + width3]];
    obs_point4 = [[obs_center + width4]];
    points = [obs_point1;obs_point2;obs_point3;obs_point4];
end
