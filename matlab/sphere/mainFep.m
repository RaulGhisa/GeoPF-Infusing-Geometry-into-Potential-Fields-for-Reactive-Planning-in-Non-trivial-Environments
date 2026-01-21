%% Script for simple kinematic control

%% Explanation about the codes:
% There are three different controllers :
% 1st - a self-writing pseudo-inverse controller (for inverse kinematic control)
% 2nd - the pseudo-inverse controller in DQ Robotic Toolbox
% 3rd - the QP controller in DQ Robotic Toolbox
% 
% And two different plot methods for trajectory plotting
% 3d plot - use the plot3 function in matlab, which is designed for 3D plot.
% DQ plot - use the plot function in DQ Robotic Toolbox, which shows the frames.
%
% Simply erase "%" in front of the codes and you can enjoy them. 

%% Initialize the file
clear;
close all;
clc;
include_namespace_dq; 

%% Initialize V-REP interface
vi = DQ_VrepInterface;
vi.disconnect_all(); 
vi.connect('127.0.0.1',19997);
vi.start_simulation();


%% Initialize VREP Robots
fep_vreprobot = FEpVrepRobot('Franka',vi);


%% Load DQ Robotics kinematics
fep = fep_vreprobot.kinematics();

% maximum joint ranges (deg): (q1..q7)
%       -166.0031 -101.0010 -166.0031 -176.0012 -166.0031  -1.0027  -166.0031
q_min = [-2.8973   -1.7628   -2.8973   -3.0718   -2.8973   -0.0175  -2.8973];

%        166.0031  101.0010  166.0031 -3.9992   166.0031   215.0024  166.0031
q_max = [ 2.8973    1.7628    2.8973  -0.0698    2.8973    3.7525    2.8973];

% the goal joint configuration (goal q)    
goal = [-pi/2.0, 0.004, 0.0, -1.57156, 0.0, 1.57075, 0.0];
% calculate the desired end-effector position according to the goal q
xd = fep.fkm(goal);


%% Change xd to the translation, create array for storing the data 
% translation is for 3D plotting and dual quaternion is for DQ plotting
xd_tran = vec3(translation(xd));
x_tran_total = [];
x_total = [];
x_ee_tran_total = [];
x_ee_total = [];


%% Set the initial error
e = zeros(8,1);
e(1) = 1.0;
disp("Starting control loop:");


%% 1st+ PD Control loop - Inverse Kinematic Control - Self-writing Controller
x_last = 0;
while(norm(e)>0.05)
   % get the joint value
   q = vi.get_joint_positions(fep_vreprobot.joint_names); 

   % forward kinematic model
   x = fep.fkm(q);

   % catch end-effector position
   x_ee = vi.get_object_pose('Franka_connection');

   % store translation and xd for plotting
   x_tran = vec3(translation(x));
   x_tran_total = [x_tran_total x_tran];
   x_total = [x_total x];
   x_ee_tran = vec3(translation(x_ee));
   x_ee_tran_total = [x_ee_tran_total x_ee_tran];
   x_ee_total = [x_ee_total x_ee];


   % control part. Inverse Kimenatic Control.
   e = vec8(x -xd);
   if x_last == 0
       e_d = vec8(DQ(0));
   else
       e_d = vec8(x_ee - x_last);
   end
   J = fep.pose_jacobian(q);
   u = -1* pinv(J) * (e);
   q = q + u;
   x_last = x_ee;

   % interact with vrep
   fep_vreprobot.send_q_to_vrep(q);
end

%% 2nd Control loop - Inverse Kinematic Control - Pseudoinverse Controller
% pseudoinverse_controller = DQ_PseudoinverseController(fep);
% pseudoinverse_controller.set_gain(1);
% pseudoinverse_controller.set_stability_threshold(0.001);
% 
% % define an alias for the method set_control_objective
% set_control_objective = @pseudoinverse_controller.set_control_objective;
% 
% % choose pose controller
% set_control_objective(ControlObjective.Pose);
% 
% while ~pseudoinverse_controller.system_reached_stable_region() 
%    task_reference = vec8(xd);
% 
%    % get the joint value
%    q = vi.get_joint_positions(fep_vreprobot.joint_names);
% 
%    x = fep.fkm(q);
%    x_ee = vi.get_object_pose('Franka_connection');
% 
%    % store translation and xd for plotting
%    x_tran = vec3(translation(x));
%    x_tran_total = [x_tran_total x_tran];
%    x_total = [x_total x];
%    x_ee = vi.get_object_pose('Franka_connection');
%    x_ee_tran = vec3(translation(x_ee));
%    x_ee_tran_total = [x_ee_tran_total x_ee_tran];
%    x_ee_total = [x_ee_total x_ee];
% 
% 
%    u = pseudoinverse_controller.compute_setpoint_control_signal(q, ...
%                                                                 task_reference);
%    q = q + 0.01*u;
% 
%    % interact with vrep
%    fep_vreprobot.send_q_to_vrep(q);
% end


%% 3rd Control loop - Inverse Kinematic Control - QP Controller
% solver = DQ_QuadprogSolver;
% QP_controller = DQ_ClassicQPController(fep, solver);
% QP_controller.set_gain(10);
% QP_controller.set_stability_threshold(0.001);
% 
% % define an alias for the method set_control_objective
% set_control_objective = @QP_controller.set_control_objective;
% 
% % choose pose controller
% set_control_objective(ControlObjective.Pose);
% 
% while ~QP_controller.system_reached_stable_region() 
%    task_reference = vec8(xd);
% 
%    % get the joint value
%    q = vi.get_joint_positions(fep_vreprobot.joint_names);
% 
%    x = fep.fkm(q);
%    x_ee = vi.get_object_pose('Franka_connection');
% 
%    % store translation and xd for plotting
%    x_tran = vec3(translation(x));
%    x_tran_total = [x_tran_total x_tran];
%    x_total = [x_total x];
%    x_ee = vi.get_object_pose('Franka_connection');
%    x_ee_tran = vec3(translation(x_ee));
%    x_ee_tran_total = [x_ee_tran_total x_ee_tran];
%    x_ee_total = [x_ee_total x_ee];
% 
% 
%    u = QP_controller.compute_setpoint_control_signal(q, task_reference);
% 
%    q = q + 0.01*u;
% 
%    % interact with vrep
%    fep_vreprobot.send_q_to_vrep(q);
% end


%% Plot 
% how many data to plot (take one point from every "interpolation" points)
interpolation = 1;

% use plot3 function in Matlab to plot the translation trajectory.
ThreeD_plot_method(xd_tran, x_tran_total, x_ee_tran_total, interpolation);

% use DQ plot function in DQ Toolbox to plot the whole trajectory (with rotation)
% DQ_plot_method(xd, x_total, x_ee_total, interpolation);


%% End V-REP
disp("Control finished");
vi.stop_simulation();
vi.disconnect();





