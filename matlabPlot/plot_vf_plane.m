function plot_vf_plane(pathpoint1, pathpoint2, pathpoint3, pathpoint4)
    plane_point1 = csvread(pathpoint1) ;
    plane_point2 = csvread(pathpoint2);
    plane_point3 = csvread(pathpoint3);
    plane_point4 = csvread(pathpoint4);
    PlotPlaneSeg(plane_point1, plane_point2, plane_point3, plane_point4);

end