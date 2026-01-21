function plot_vf_robot_traj(robot_pos, marker)

    if nargin < 2  % Check if the marker argument is provided
        marker = 'r.';  % Default marker style is red dots
    end
    plot3(robot_pos(:,1),robot_pos(:,2),robot_pos(:,3),"MarkerFaceColor", marker,"MarkerFaceColor","r", 'LineWidth', 5);
    hold on

end
