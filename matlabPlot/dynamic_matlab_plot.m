clear 
clc
dt = 0.01* 0.1;
moving_steps = 10;
moving_interval = 100;
%path_base = "/home/yuhe/Desktop/project/3-GeometricPrimitives/RealExp/GeometricPrimitives/data";
path_base =  "/home/yuhe/Desktop/project/3-GeometricPrimitives/cpp/data/compdynamic_simple_folder_31";
path_pub = path_base + "/";%"/pub_";
path_sub = path_base + "/";%"/sim_";
%figure();

% 创建一个VideoWriter对象，设置视频文件名和帧率
outputVideo = VideoWriter('output_video.avi', 'Uncompressed AVI');
outputVideo.FrameRate = 30; % 设置帧率为30fps
open(outputVideo); % 打开视频写入对象

% 设置图形窗口的尺寸
figure;
set(gcf, 'Position', [100, 100, 560, 420]); 
hold on;
i_circle = 0;
i_line = 0;
i_plane = 0;
i_cubic = 0;
i_cylinder = 0;
folderPath = path_base + "/moving";
% List all files in the directory
files = dir(folderPath);

% cubic_point1 = csvread(path_pub + "cubicpoint1.csv");
% cubic_point2 = csvread(path_pub + "cubicpoint2.csv");
% cubic_point3 = csvread(path_pub + "cubicpoint3.csv");
% cubic_point4 = csvread(path_pub + "cubicpoint4.csv");
% cubic_point5 = csvread(path_pub + "cubicpoint5.csv");
% cubic_point6 = csvread(path_pub + "cubicpoint6.csv");
% cubic_point7 = csvread(path_pub + "cubicpoint7.csv");
% cubic_point8 = csvread(path_pub + "cubicpoint8.csv");
robot_pos = csvread(path_sub + "trajectory.csv");
line_start = csvread(path_pub + "linestart.csv");
line_end = csvread(path_pub + "lineend.csv");
plane_point1 = csvread(path_pub + "planepoint1.csv") ;
plane_point2 = csvread(path_pub + "planepoint2.csv");
plane_point3 = csvread(path_pub + "planepoint3.csv");
plane_point4 = csvread(path_pub + "planepoint4.csv");
moving_line = line_start;
moving_line = zeros(size(moving_line));  % Set 'moving_line' matrix to zero, preserving its size
moving_cylinder = csvread(path_pub + "cylinderstart.csv");
moving_cylinder = zeros(size(moving_cylinder));  % Set 'moving_line' matrix to zero, preserving its size
moving_plane = csvread(path_pub + "planepoint1.csv");
moving_plane = zeros(size(moving_plane));  % Set 'moving_line' matrix to zero, preserving its size

disp(['Folder Path: ', folderPath]);

cylinder_start = csvread(path_pub + "cylinderstart.csv");
cylinder_end = csvread(path_pub + "cylinderend.csv");
cylinder_rad = csvread(path_pub + "cylinderrad.csv");


% Loop through each file in the directory
for k = 1:length(files)
    fileName = files(k).name;
    
    % Skip "." and ".."
    if strcmp(fileName, '.') || strcmp(fileName, '..')
        continue;
    end
    
    % Check if the file name contains specific keywords
    if contains(fileName, 'circle_moving')
        i_circle = i_circle + 1;
    elseif contains(fileName, 'line_moving')
        i_line = i_line + 1;
    elseif contains(fileName, 'plane_moving')
        i_plane = i_plane + 1;
    elseif contains(fileName, 'cubic_moving')
        i_cubic = i_cubic + 1;
    elseif contains(fileName, 'cylinder_moving')
        i_cylinder = i_cylinder + 1;
    end
end

% Initialize matrices to hold the moving object data
moving_object_circle = NaN(moving_steps, 3 * i_circle);
moving_object_line = NaN(moving_steps, 3 * i_line);
moving_object_plane = NaN(moving_steps, 3 * i_plane);
moving_object_cubic = NaN(moving_steps, 3 * i_cubic);
moving_object_cylinder = NaN(moving_steps, 3 * i_cylinder);

% Load data for circle objects
for j = 1:i_circle
    path_file = fullfile(folderPath, sprintf('circle_movingobject_%d.csv', j-1));
    matrix_obj = csvread(path_file);
    moving_object_circle(:, (j-1)*3+1:j*3) = matrix_obj;
end

% Load data for line objects
for j = 1:i_line
    path_file = fullfile(folderPath, sprintf('line_movingobject_%d.csv', j-1));
    matrix_obj = csvread(path_file);
    moving_object_line(:, (j-1)*3+1:j*3) = matrix_obj;
end

% Load data for plane objects
for j = 1:i_plane
    path_file = fullfile(folderPath, sprintf('plane_movingobject_%d.csv', j-1));
    matrix_obj = csvread(path_file);
    moving_object_plane(:, (j-1)*3+1:j*3) = matrix_obj;
end

% Load data for cubic objects
for j = 1:i_cubic
    path_file = fullfile(folderPath, sprintf('cubic_movingobject_%d.csv', j-1));
    matrix_obj = csvread(path_file);
    moving_object_cubic(:, (j-1)*3+1:j*3) = matrix_obj;
end

% Load data for cylinder objects
for j = 1:i_cylinder
    path_file = fullfile(folderPath, sprintf('cylinder_movingobject_%d.csv', j-1));
    matrix_obj = csvread(path_file);
    moving_object_cylinder(:, (j-1)*3+1:j*3) = matrix_obj;
end

disp('All files processed.');

plot3(robot_pos(:,1),robot_pos(:,2),robot_pos(:,3), 'Color',"#0072BD", 'LineWidth', 10);
hold on

for t = 1:100%size(robot_pos, 1)

    if mod(t, moving_interval) ~= 0
        continue; % Skip the rest of the loop for this iteration
    end

    %cla; %
    
    %% plot trajectory
    
    

    
    %% plot workspace boundary
    %workspaceboundary = csvread(path_pub + "workspaceboundary.csv");
    %plot_vf_workspace_bound(workspaceboundary);
    
    %
    % plot circles
    %plot_vf_sphere(path_pub + "sphere.csv", path_pub + "sphererad.csv")
    
    %% line plotting
    %plot_vf_line(path_pub + "linestart.csv", path_pub + "lineend.csv")
    
    for j=1:size(line_start,1)
        if j <= i_line
            if t < moving_steps * moving_interval && mod(t, moving_interval) == 0
                moving_line(j,:) = moving_line(j,:) + moving_object_line(t/moving_interval, ((j-1)*3+1):((j-1)*3 +3)) * dt

            end
        end
        disp("moving line outside")
        moving_line(j,:)

        line_start(j,:) = line_start(j,:) + moving_line(j,:);
        line_end(j,:) = line_end(j,:) + moving_line(j,:);
        plot3([line_start(j,1), line_end(j,1)], [line_start(j,2), line_end(j,2)], [line_start(j,3), line_end(j,3)], "-");
        hold on
    end
    
    %% plane plotting
    %plot_vf_plane(path_pub + "planepoint1.csv", path_pub + "planepoint2.csv", path_pub + "planepoint3.csv", path_pub + "planepoint4.csv")
    
    for j=1:size(plane_point1,1)
        if j <= i_plane
            if t < moving_steps * moving_interval && mod(t, moving_interval) == 0
                moving_plane(j,:) = moving_plane(j,:) + moving_object_plane(t/moving_interval, ((j-1)*3+1):((j-1)*3 +3)) * dt
            end
        end
        plane_point1(j,:) = plane_point1(j,:)+ moving_cylinder(j,:);
        plane_point2(j,:) = plane_point2(j,:)+ moving_cylinder(j,:);
        plane_point3(j,:) = plane_point3(j,:)+ moving_cylinder(j,:);
        plane_point4(j,:) = plane_point4(j,:)+ moving_cylinder(j,:);


        % Define the four points of the plane
        x = [plane_point1(j,1), plane_point2(j,1), plane_point3(j,1), plane_point4(j,1)]; % X-coordinates of the corners
        y = [plane_point1(j,2), plane_point2(j,2), plane_point3(j,2), plane_point4(j,2)]; % Y-coordinates of the corners
        z = [plane_point1(j,3), plane_point2(j,3), plane_point3(j,3), plane_point4(j,3)]; % Z-coordinates of the corners
        
        % Plot the plane and fill with a dark color
        fill3(x, y, z, [0.1, 0.1, 0.1], 'FaceAlpha', 0.8); % Dark gray color, 80% opacity
        hold on;
        
        % Add edges for better visualization (optional)
        plot3(x, y, z, 'k-', 'LineWidth', 2);

    end

    %% Cube Plotting
    % plot_vf_cubic(path_pub + "cubicpoint1.csv", path_pub + "cubicpoint2.csv", path_pub + "cubicpoint3.csv", path_pub + "cubicpoint4.csv", path_pub + "cubicpoint5.csv", path_pub + "cubicpoint6.csv", path_pub + "cubicpoint7.csv", path_pub + "cubicpoint8.csv" )
  
    % PlotPlaneSeg(cubic_point1, cubic_point2, cubic_point3, cubic_point4);
    % PlotPlaneSeg(cubic_point5, cubic_point6, cubic_point7, cubic_point8);
    % PlotPlaneSeg(cubic_point2, cubic_point6, cubic_point7, cubic_point3);
    % PlotPlaneSeg(cubic_point5, cubic_point1, cubic_point4, cubic_point8);
    % PlotPlaneSeg(cubic_point4, cubic_point3, cubic_point7, cubic_point8);
    % PlotPlaneSeg(cubic_point1, cubic_point2, cubic_point6, cubic_point5);
    %% Plot cylinder
    
    for j = 1:size(cylinder_start, 1)
    if j <= i_cylinder
        if t < moving_steps * moving_interval && mod(t, moving_interval) == 0
            moving_cylinder(j, :) = moving_cylinder(j, :) + moving_object_cylinder(t / moving_interval, ((j - 1) * 3 + 1):((j - 1) * 3 + 3)) * dt;
        end
    end
    
    % Update cylinder position
    cylinder_start(j, :) = cylinder_start(j, :) + moving_cylinder(j, :);
    cylinder_end(j, :) = cylinder_end(j, :) + moving_cylinder(j, :);
    
    % Compute cylinder axis direction
    direction = cylinder_end(j, :) - cylinder_start(j, :);
    direction = direction / norm(direction);
    
    % Create cylinder surface
    [X, Y, Z] = cylinder(cylinder_rad(j, :), 50); % 50 points for smoothness
    Z = Z * norm(cylinder_end(j, :) - cylinder_start(j, :)); % Scale height
    
    % Rotate cylinder to align with its direction
    v = [0, 0, 1]; % Default cylinder direction in MATLAB
    rotation_axis = cross(v, direction);
    rotation_angle = acos(dot(v, direction));
    if norm(rotation_axis) > 0
        rotation_axis = rotation_axis / norm(rotation_axis);
    else
        rotation_axis = [1, 0, 0];
    end
    
    % Create rotation matrix
    R = axang2rotm([rotation_axis, rotation_angle]);
    
    % Apply rotation
    rotated_points = R * [X(:)'; Y(:)'; Z(:)'];
    X_rot = reshape(rotated_points(1, :), size(X)) + cylinder_start(j, 1);
    Y_rot = reshape(rotated_points(2, :), size(Y)) + cylinder_start(j, 2);
    Z_rot = reshape(rotated_points(3, :), size(Z)) + cylinder_start(j, 3);
    
    % Plot cylinder surface
    surf(X_rot, Y_rot, Z_rot, 'FaceColor', 'k', 'EdgeColor', 'none');
    hold on;
    
    % Plot start and end caps
    theta = linspace(0, 2 * pi, 50);
    xc = cylinder_rad(j) * cos(theta);
    yc = cylinder_rad(j) * sin(theta);
    zc = zeros(size(xc));
    circle_points = [xc; yc; zc];
    
    rotated_caps = R * circle_points;
    
    % Start cap
    patch(rotated_caps(1, :) + cylinder_start(j, 1), rotated_caps(2, :) + cylinder_start(j, 2), rotated_caps(3, :) + cylinder_start(j, 3), 'k');
    
    % End cap
    patch(rotated_caps(1, :) + cylinder_end(j, 1), rotated_caps(2, :) + cylinder_end(j, 2), rotated_caps(3, :) + cylinder_end(j, 3), 'k');
    
    % Mark cylinder start and end points
    plot3(cylinder_start(j, 1), cylinder_start(j, 2), cylinder_start(j, 3), '*r');
    plot3(cylinder_end(j, 1), cylinder_end(j, 2), cylinder_end(j, 3), '*b');

        hold on
    end
    %% primilinaries
    %disp("iteractions")
    axis equal;
    %title("Path towards goal");
    %hold off
    grid on
    %toc
    hold on
    title(['Frame: ', num2str(t)]); % 显示当前帧数
    view(3);

    alpha(.5)

    %camorbit(1, 0, 'camera');
    
    frame = getframe(gcf); % 获取当前图形窗口的帧
    frame_resized = imresize(frame.cdata, [560, 420]);
    writeVideo(outputVideo, frame_resized); % 将帧写入视频文
end


function S = skew_symmetric(v)
    S = [ 0, -v(3), v(2); 
          v(3), 0, -v(1); 
         -v(2), v(1), 0];
end