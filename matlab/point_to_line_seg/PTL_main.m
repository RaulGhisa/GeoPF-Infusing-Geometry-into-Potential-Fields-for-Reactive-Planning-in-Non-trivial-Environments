clc, clear, close all
include_namespace_dq

%% Initialize V-REP interface
vi = DQ_VrepInterface;
vi.disconnect_all(); 
vi.connect('127.0.0.1',19997);
vi.start_simulation();


%% Initialize VREP Robots
fep_vreprobot = FEpVrepRobot('Franka',vi);

%% Load DQ Robotics kinematics
fep = fep_vreprobot.kinematics();




%% Set the starting and parameters

% START = [-1.4981 1.0211 0.3888];
% GOAL = [-0.7905 0.5477 0.7587];

q_start = vi.get_joint_positions(fep_vreprobot.joint_names); 
START = vec3(translation(fep.fkm(q_start)))';

robot_orientation = rotation(fep.fkm(q_start));

% the goal joint configuration (goal q)    
q_goal = [-pi/2.0, 0.004, 0.0, -1.57156, 0.0, 1.57075, 0.0];
% calculate the desired end-effector position according to the goal q
GOAL = START + [0 -0.2 0];% vec3(translation(fep.fkm(q_goal)))';


robot_pos = START;
robot_vel = zeros(1,3);
goal = GOAL;
ROBOT_MASS = 2.0;
time_step = 0.05;
V_MAX = 0.05;

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

obs_rad = 0.005;

%obs_pos = [Obstacles1; Obstacles2; Obstacles3; Obstacles4; Obstacles5;...
%    Obstacles6; Obstacles7; Obstacles8; Obstacles9];
obs_center = (GOAL - START) * 0.5 + START;
obs_center2 = (GOAL - START) * 0.3 + START;
obs_start = [[obs_center + [0.1 0 0.1]];[obs_center2 + [0.1  0  0.2]];[obs_center2 + [0  0  0.2]]];
obs_end = [[obs_center - [0.1 0 0.1]]; [obs_center2 + [0 0 -0.1]];[obs_center2 + [0 0 -0.1]]];
% obs_pos = [[0 0 0]];
%% Gains 
k_line = 0.5;
k_attr = 0.2;

%% Attractor
att_force = AttractorForces(goal, robot_pos, k_attr);

% Distance to goal 
d = norm(goal - robot_pos);

i = 1;

x_last = 0;

%% Control Loop
while d > 0.01
    q = vi.get_joint_positions(fep_vreprobot.joint_names); 

    % forward kinematic model
    x_curr = fep.fkm(q);

    % catch end-effector position
    x_ee = vi.get_object_pose('Franka_connection');



    % reseting the forces so that residuals are removed
    force = zeros(1,3);

    % Add the attractor forces
    att_force = AttractorForces(goal, robot_pos, k_attr);
    force = force + att_force;

    for j = 1:size(obs_start,1)

      % % Add circular forces for each obstacle
      circ_force = PointToLineSegForces(obs_start(j,:), obs_end(j,:), robot_pos, robot_vel, goal, k_line);

      % Add line segment forces for each obstacle
      % circ_force = PointToLineSegment(obs_pos(j,:), obs_rad, robot_pos, robot_vel, goal, k_line);

      force = force + circ_force;

    end

    acc = force / ROBOT_MASS;

    robot_acc = acc;
    robot_vel = robot_vel + acc * time_step;
    vel_norm = norm(robot_vel);
    
    if vel_norm > V_MAX
        robot_vel = robot_vel * (V_MAX/vel_norm); 
    end
    
    robot_pos = robot_pos + 0.5 * acc * time_step * time_step+ robot_vel * time_step;

    xd = robot_orientation + DQ.E * 0.5 *  DQ(robot_pos) * robot_orientation;

    e = vec8(x_curr -xd);
    if x_last == 0
        e_d = vec8(DQ(0));
    else
        e_d = vec8(x_ee - x_last);
    end
    e_d = vec8(x_ee - x_last);

    J = fep.pose_jacobian(q);
    u = -0.1 * pinv(J) * (e);
    q = q + u;
    x_last = x_ee;
    fep_vreprobot.send_q_to_vrep(q);
    
    q = vi.get_joint_positions(fep_vreprobot.joint_names); 
    robot_pos_last = robot_pos;
    robot_pos_new = vec3(translation(fep.fkm(q)))'
    goal
    %robot_vel = (robot_pos - robot_pos_last);
    %robot_pos


    
    % If the robot is too close to the goal slow it down
    % if d < 0.1
    %     robot_vel = robot_vel * (10*d);
    % end  
    
    % Update the distance to goal
    d = norm(goal - robot_pos_new);
    d


    x(i) = robot_pos_new(1);
    xdot(i) = robot_vel(1);
    %xddot(i) = robot_acc(1);
    y(i) = robot_pos_new(2);
    ydot(i) = robot_vel(2);
    %yddot(i) = robot_acc(2);
    z(i) = robot_pos_new(3);
    zdot(i) = robot_vel(3);
    %zddot(i) = robot_acc(3);
    t(i) = i * time_step;
    
    i = i + 1;
end

vi.stop_simulation();

%% Visualization and Plots

figure();
[X,Y,Z] = sphere;
axis equal;
plot3(x, y, z, "r.");
title("Path towards goal");
hold on

for j=1:size(obs_start,1)
    plot3([obs_start(j,1), obs_end(j,1)], [obs_start(j,2), obs_end(j,2)], [obs_start(j,3), obs_end(j,3)], "-");
    hold on
end