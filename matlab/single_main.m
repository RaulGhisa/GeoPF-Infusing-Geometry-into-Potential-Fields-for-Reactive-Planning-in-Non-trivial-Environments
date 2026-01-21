clc, clear, close all

%% Set the starting and parameters

tic

long = 10;
Heu_k_attr = cell(1,10);%0.1;
Heu_k_circ = cell(1,10);
Heu_k_circ_inside = cell(1,3);
Heu_k_line = cell(1,3);
Heu_k_plane = cell(2,3);
Heu_plane_dist = cell(2,3);
Heu_k_plane_line = cell(1,3);
Heu_k_cube = cell(1,3);


k=cell(10,1);


path_len = 0

for i = 1:1


    traj1 = []
    traj2 = []
    traj3 = []
    traj4 = []
    traj5 = []
    traj6 = []
    traj7 = []
    traj8 = []
    traj9 = []
    traj10 = []




   

    T = [1000000,1000000,1000000];
    
    %i_index = 1;

    START = [0 0 0];
    GOAL = START + [-0.05 0.55 0.1];
    
    robot_pos = START;
    robot_vel = zeros(1,3);
    goal = GOAL;
    ROBOT_MASS = 1.0;
    time_step = 0.01;
    V_MAX = 0.04;

    
    
    %% Obstacle positions and radius
    Obstacles1 = [-1.1494 0.7725 0.5802];
    Obstacles2 = [-0.9282 0.6825 0.6612];
    Obstacles3 = [-1.005 0.7524 0.6102];
    Obstacles4 = [-1.005 0.7224 0.6102];
    Obstacles5 = [-1.005 0.7424 0.6102];
    Obstacles6 = [-1.005 0.7624 0.6102];
    Obstacles7 = [-1.1494 0.7725 0.5802];
    Obstacles8 = [-0.9282 0.6825 0.6912];
    Obstacles9 = [-1.005 0.7524 0.6502];
    obs_center = (GOAL - START) * 0.6 + START ;
    obs_rad = 0.01;
    obs_pos = [[(GOAL-START) * 0.5 + START];[(GOAL-START) * 0.3 + START]+[-0.1 0.2 0.02];[(GOAL-START) * 0.3 + START]-[0 0.1 0.02];[(GOAL-START) * 0.2 + START];[(GOAL-START) * 0.7 + START]+[ -0.15 -0 0.05];];
    

    %% the boundary of the workspace
    obs_boundary = [0 0 0];
    obs_boundary_rad = 0.8;
    
    
    %% the line obstacle
    obs_center = (GOAL - START) * 0.5 + START;
    obs_center2 = (GOAL - START) * 0.3 + START;
    obs_start = [[obs_center + [0 0 0.1]];[obs_center2 + [0.1  0  0.2]];[obs_center2 + [0  0  0.2]]];
    obs_end = [[obs_center - [0 0 0.1]]; [obs_center2 + [0 0 -0.1]];[obs_center2 + [0 0 -0.1]]];
    
    
    %% the plane obstacle
    obs_center = [(GOAL - START) * 0.5 + START + [0, 0, -0.05];(GOAL - START) * 0.3 + START + [0.005, 0, 0.02]];
    width =  [0.2;0.15];
    plane_direction = ["Y"; "Z"];
    
    %obs_center = [(GOAL - START) * 0.6 + START + [0.005, 0, 0.02];(GOAL - START) * 0.3 + START + [0, 0, 0.01];(GOAL - START) * 0.5 + START + [0, 0, -0.05]];
    %width =  [0.15;0.05;0.2];
    %plane_direction = ["Y"; "Y"; "Z"];
    
    obs_point1 = zeros(size(obs_center));
    obs_point2 = zeros(size(obs_center));
    obs_point3 = zeros(size(obs_center));
    obs_point4 = zeros(size(obs_center));
    
    for j = 1:size(obs_center,1)
        points = PlanePoints(obs_center(j,:), width(j,:), plane_direction(j,:));
        obs_point1(j,:) = points(1,:);
        obs_point2(j,:) = points(2,:);
        obs_point3(j,:) = points(3,:);
        obs_point4(j,:) = points(4,:);
    end

    
    %% Cube obstacle
    obs_center_cube = [0, 0.3, 0.1];%(GOAL - START) * 0.6 + START + [0, 0.05, 0.01]];
    width_cube = 0.1;
    
    %cube = CubeSeg(obs_center_cube, width_cube);
    
    
    
    % [obs_center2 + [width2 0 -width2]];[obs_center3 + [width3 -width3 0]]];
    
    %for i_index=1:100
    
    
    % Low Pass Filter
    % T_s: sampling time = 0.001 
    Ts = 0.01;
    % F_c: cut_off frequency = 1
    fc = 1;
    %a = T_s / (T_s + RC) = 2 * pi * f_c * T_s / (2 * pi * f_c * T_s + 1)
    a = 2 * 3.14 * fc * Ts / (2 * 3.14 * fc * Ts + 1);
    % Low Pass Filter: y_n = a x_n + (1 - a) * y_{n-1} = y_{n-1} + a * (x_n - y_{n-1})
    robot_pos_last = robot_pos;
    

    %% implement the filter for trajectory!!! 



    %% Gains 
    k_attr = 1;%0.1;
    k_circ = 0.1;
    k_circ_inside = 0.1;
    k_line = 0.1;
    k_plane = [0.1;0.1];
    plane_dist = [0.02;0.015];
    k_plane_line = 1;
    k_cube = 0.1;
    
    %% Attractor
    att_force = AttractorForces(goal, robot_pos, k_attr);
    
    % Distance to goal 
    d = norm(goal - robot_pos);
    
    ii = 1;
    
    vector_x = [];
    vector_y = [];
    vector_z = [];

    x = [];
    xdot = [];
    xddot = [];
    y = [];
    ydot = [];
    yddot = [];
    z = [];
    zdot = [];
    zddot = [];
    t = [];

    GOAL = START + [-0.1 0.5 0.3];
    force_store = [];

    force_store1 = zeros(1,3);
    force_store2 = zeros(1,3);
    plane_force_last1 = zeros(1,3);
    plane_force_last2 = zeros(1,3);
            
    %% Control Loop
    while d > 0.01
    
        % reseting the forces so that residuals are removed
        force = zeros(1,3);
        %robot_pos = round(robot_pos,4);
    
        % Add the attractor forces
        att_force = AttractorForces(goal, robot_pos, k_attr);
        ii
       
        force = force + att_force;
        
        
    
        % %% sphere
        % for j = 1:size(obs_pos,1)
        %   circ_force = CurrForces(obs_pos(j,:), obs_rad, robot_pos, robot_vel, goal, k_circ);
        %   force = force + circ_force;
        % end
        % insidecirc_force = insideCurrForces(obs_boundary, obs_boundary_rad, robot_pos, robot_vel, goal, k_circ_inside);
        % force = force + insidecirc_force;
        % 
        % %% line
        % for j = 1:size(obs_start,1)
        %   line_force = PointToLineSegForces(obs_start(j,:), obs_end(j,:), robot_pos, robot_vel, goal, k_line);
        %   force = force + line_force;
        % end
        % 
        % %% plane
        %  for j = 1:size(obs_center,1)
        %   plane_force = PointToPlaneSegForces(obs_center(j,:), width(j,:), plane_direction(j,:), robot_pos, robot_vel, goal, k_plane(j,:), plane_dist(j,:), k_plane_line)
        %   force = force + plane_force;
        %  end
    
        for j = 1:size(obs_center_cube,1)
            cube_force = PointToCubeForces(obs_center_cube(j,:), width_cube(j,:), robot_pos, robot_vel, goal, k_cube(j,:))
            force = force + cube_force;
        end


        
        
        
        robot_pos
        
    
        acc = force / ROBOT_MASS;
    
        robot_acc = acc;
        robot_vel = robot_vel + acc * time_step;
        vel_norm = norm(robot_vel);
        
        if vel_norm > V_MAX
           robot_vel = robot_vel * (V_MAX/vel_norm); 
        end
        if d < 0.1
            robot_vel = robot_vel * (10*d);
        end 
        robot_pos_last = robot_pos;
        
        robot_pos = robot_pos + 0.5 * acc * time_step * time_step+ robot_vel * time_step;

        path_len = path_len + norm(robot_pos - robot_pos_last);
    
        %robot_pos = robot_pos_last + a * (robot_pos - robot_pos_last);
    
        %robot_pos_last = robot_pos;
        
        % If the robot is too close to the goal slow it down
        % robot_vel
         
        
        % Update the distance to goal
            
        d = norm(goal - robot_pos);
    
        x(ii) = robot_pos(1);
        xdot(ii) = robot_vel(1);
        xddot(ii) = robot_acc(1);
        y(ii) = robot_pos(2);
        ydot(ii) = robot_vel(2);
        yddot(ii) = robot_acc(2);
        z(ii) = robot_pos(3);
        zdot(ii) = robot_vel(3);
        zddot(ii) = robot_acc(3);
        t(ii) = ii * time_step;
        
        ii = ii + 1;

        if i == 1
            traj1(:,ii) = robot_pos;        
        elseif i == 2
            traj2(:,ii) = robot_pos;
        elseif i == 3
            traj3(:,ii) = robot_pos;
        elseif i == 4
            traj4(:,ii) = robot_pos;
        elseif i == 5
            traj5(:,ii) = robot_pos;
        elseif i == 6
            traj6(:,ii) = robot_pos;
        elseif i == 7
            traj7(:,ii) = robot_pos;
        elseif i == 8
            traj8(:,ii) = robot_pos;
        elseif i == 9
            traj9(:,ii) = robot_pos;
        elseif i == 10
            traj10(:,ii) = robot_pos;
        end


        %ii

       
        
    end

    if i == 1
        k{i} = traj1;
    elseif i == 2
        k{i} = traj2;
    elseif i == 3
        k{i} = traj3;
    elseif i == 4
        k{i} = traj4;
    elseif i == 5
        k{i} = traj5;
    elseif i == 6
        k{i} = traj6;
    elseif i == 7
        k{i} = traj7;
    elseif i == 8
        k{i} = traj8;
    elseif i == 9
        k{i} = traj9;
    elseif i == 10
        k{i} = traj10;
    end


    %i_index = i_index + 1;

    %i_index
    


    %if ii < T
    %    Heu_k_attr = k_attr;%0.1;
    %    Heu_k_circ = k_circ;
    %    Heu_k_circ_inside = k_circ_inside;
    %    Heu_k_line = k_line;
    %    Heu_k_plane = k_plane;    
    %    Heu_plane_dist = plane_dist;
    %    Heu_k_plane_line = k_plane_line;
    %    Heu_k_cube = k_cube;
    %    T = ii;
    %end

    %if ii < T{2} && ii > T{1}
    %    Heu_k_attr{2} = k_attr;%0.1;
    %    Heu_k_circ{2} = k_circ;
    %    Heu_k_circ_inside{2} = k_circ_inside;
    %    Heu_k_line{2} = k_line;
    %    Heu_k_plane{:,2}= k_plane;    
    %    Heu_plane_dist{:,2} = plane_dist;
    %   Heu_k_plane_line{2} = k_plane_line;
    %    Heu_k_cube{2} = k_cube;
    %    T{2} = ii;
    %end

    %if ii < T{3} && ii > T{2}
    %    Heu_k_attr{3} = k_attr;%0.1;
    %    Heu_k_circ{3} = k_circ;
    %    Heu_k_circ_inside{3} = k_circ_inside;
    %    Heu_k_line{3} = k_line;
    %    Heu_k_plane{:,3} = k_plane;    
    %    Heu_plane_dist{:,3} = plane_dist;
    %    Heu_k_plane_line{3} = k_plane_line;
    %    Heu_k_cube{3} = k_cube;
    %    T{3} = ii;
    %end

end



%end

%% Visualization and Plots

figure();
[X,Y,Z] = sphere;
for j=1:size(obs_pos,1)
    surf(X * obs_rad + obs_pos(j,1),Y * obs_rad + obs_pos(j,2), Z * obs_rad + obs_pos(j,3));
    hold on
end
surf(X * obs_boundary_rad + obs_boundary(1),Y * obs_boundary_rad + obs_boundary(2), Z * obs_boundary_rad + obs_boundary(3),'FaceAlpha',.3,'EdgeColor','none');
hold on


% %% line plotting
% for j=1:size(obs_start,1)
%     plot3([obs_start(j,1), obs_end(j,1)], [obs_start(j,2), obs_end(j,2)], [obs_start(j,3), obs_end(j,3)], "-");
%     hold on
% end
% 
% %% Plane Segment Plotting
% PlotPlaneSeg(obs_point1, obs_point2, obs_point3, obs_point4);

%% Cube Plotting
PlotCubeSeg(obs_center_cube, width_cube);


disp("iteractions")
axis equal;
plot3(x, y, z, "r.");
title("Path towards goal");
%hold off
toc