function plot_vf_cubic(pathpoint1, pathpoint2, pathpoint3, pathpoint4, pathpoint5, pathpoint6, pathpoint7, pathpoint8)
    cubic_point1 = csvread(pathpoint1);
    cubic_point2 = csvread(pathpoint2);
    cubic_point3 = csvread(pathpoint3);
    cubic_point4 = csvread(pathpoint4);
    cubic_point5 = csvread(pathpoint5);
    cubic_point6 = csvread(pathpoint6);
    cubic_point7 = csvread(pathpoint7);
    cubic_point8 = csvread(pathpoint8);
    PlotPlaneSeg(cubic_point1, cubic_point2, cubic_point3, cubic_point4);
    PlotPlaneSeg(cubic_point5, cubic_point6, cubic_point7, cubic_point8);
    PlotPlaneSeg(cubic_point2, cubic_point6, cubic_point7, cubic_point3);
    PlotPlaneSeg(cubic_point5, cubic_point1, cubic_point4, cubic_point8);
    PlotPlaneSeg(cubic_point4, cubic_point3, cubic_point7, cubic_point8);
    PlotPlaneSeg(cubic_point1, cubic_point2, cubic_point6, cubic_point5);

end