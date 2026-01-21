
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function att_force = AttractorForces(goal, robot_pos, k_attr)

% Function that computes the attractive forces

goal_vec = (goal - robot_pos);
goal_vec = goal_vec / norm(goal_vec);
att_force = k_attr * goal_vec;

end