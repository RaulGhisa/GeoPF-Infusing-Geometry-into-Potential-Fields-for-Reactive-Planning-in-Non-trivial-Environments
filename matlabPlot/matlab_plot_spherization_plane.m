clear 
clc
path_base =  "/home/yuhe/Desktop/project/3-GeometricPrimitives" + ...
    "/cpp/data/line_folder_9/spherization_";
% robot_pos = csvread("/home/yuhe/Desktop/proje" + ...
%     "ct/3-GeometricPrimitives/cpp/data/lin" + ...
%     "e_simple_folder_13/spherization_fast_1_r_0." + ...
%     "01_" + "trajectory.csv");
% plot_vf_robot_traj(robot_pos);

%path_base = "/home/yuhe/Desktop/project/3-GeometricPrimitives/cpp-spherization/data/"
figure();
[X,Y,Z] = sphere;

%% plot trajectory

robot_pos = csvread(path_base + "trajectory.csv");
%plot_vf_robot_traj(robot_pos, "r");
plot3(robot_pos(:,1),robot_pos(:,2),robot_pos(:,3),'Color', '#77AC30', 'LineWidth', 10);
hold on
legend("CF-0.01")
%% plot workspace boundary
%workspaceboundary = csvread(path_base + "workspaceboundary.csv");
%plot_vf_workspace_bound(workspaceboundary);

%%% plot circles
%plot_vf_sphere(path_base + "sphere.csv", path_base + "sphererad.csv")

%% line plotting
%plot_vf_line(path_base + "linestart.csv", path_base + "lineend.csv")

%% plane plotting
%plot_vf_plane(path_base + "planepoint1.csv", path_base + "planepoint2.csv", path_base + "planepoint3.csv", path_base + "planepoint4.csv")
% 
% %% Cube Plotting
% plot_vf_cubic(path_base + "cubicpoint1.csv", path_base + "cubicpoint2.csv", path_base + "cubicpoint3.csv", path_base + "cubicpoint4.csv", path_base + "cubicpoint5.csv", path_base + "cubicpoint6.csv", path_base + "cubicpoint7.csv", path_base + "cubicpoint8.csv" )
% 
% %% Plot cylinder
% plot_vf_cylinder(path_base + "cylinderstart.csv", path_base + "cylinderend.csv", path_base + "cylinderrad.csv")


%% primilinaries
disp("iteractions")
axis equal;
title("Path towards goal - spherization");
%hold off
%grid on
ylim([-1 1]) 
xlim([-0.2 0.2])
zlim([-0.2 0.2]) 
toc
