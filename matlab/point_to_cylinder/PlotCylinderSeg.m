function PlotCubeSeg(obs_center, width)

    cube = CubeSeg(obs_center,width);        
    for i=1:size(cube,1)
        obs_point1 = cube(i,1:3);
        obs_point2 = cube(i,4:6);
        obs_point3 = cube(i,7:9);
        obs_point4 = cube(i,10:12);
        for j=1:size(obs_point1,1)
            plot3([obs_point1(j,1), obs_point2(j,1)], [obs_point1(j,2), obs_point2(j,2)], [obs_point1(j,3), obs_point2(j,3)], "-");
            hold on
        end
        
        for j=1:size(obs_point2,1)
            plot3([obs_point3(j,1), obs_point2(j,1)], [obs_point3(j,2), obs_point2(j,2)], [obs_point3(j,3), obs_point2(j,3)], "-");
            hold on
        end
        
        for j=1:size(obs_point4,1)
            plot3([obs_point3(j,1), obs_point4(j,1)], [obs_point3(j,2), obs_point4(j,2)], [obs_point3(j,3), obs_point4(j,3)], "-");
            hold on
        end
        
        for j=1:size(obs_point1,1)
            plot3([obs_point1(j,1), obs_point4(j,1)], [obs_point1(j,2), obs_point4(j,2)], [obs_point1(j,3), obs_point4(j,3)], "-");
            hold on
        end
    end
end
