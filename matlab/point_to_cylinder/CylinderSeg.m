function plane = CubeSeg(obs_center, width)
    obs_center = [obs_center + [0 0 width];obs_center + [0 0 -width];obs_center + [0 width 0];obs_center + [0 -width 0];obs_center + [width 0 0];obs_center + [-width 0 0]];
    plane_direction = ["Z";"Z";"Y";"Y";"X";"X"];
    
    obs_point1 = zeros(size(obs_center));
    obs_point2 = zeros(size(obs_center));
    obs_point3 = zeros(size(obs_center));
    obs_point4 = zeros(size(obs_center));
    
    for j = 1:size(obs_center,1)
        points = PlanePoints(obs_center(j,:), width, plane_direction(j,:));
        obs_point1(j,:) = points(1,:);
        obs_point2(j,:) = points(2,:);
        obs_point3(j,:) = points(3,:);
        obs_point4(j,:) = points(4,:);
    end
    plane = [obs_point1,obs_point2,obs_point3,obs_point4];
end
