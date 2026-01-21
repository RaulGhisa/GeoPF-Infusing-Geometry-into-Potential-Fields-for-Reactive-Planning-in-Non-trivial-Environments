
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function pont_to_cube_force = PointToCubeForces(obs_center, width, robot_pos, robot_vel, goal, k_plane)
%width = width + 0.03;
cube = CubeSeg(obs_center,width);  
dist_and_index = PointToCubecomp_dist(robot_pos, obs_center, width, goal);
dist = dist_and_index(1,1:3);
dist_index = dist_and_index(1,4)
robot_obs = dist_and_index(1,5:7);
dist_obs = norm(robot_obs);

obs_point1 = cube(dist_index,1:3);
obs_point2 = cube(dist_index,4:6);
obs_point3 = cube(dist_index,7:9);
obs_point4 = cube(dist_index,10:12);

pont_to_cube_force = zeros(1,3);

dist_obs = dist_obs
% Function that computes the circular force    
if dist_obs < 0.1
   if norm(robot_vel) ~= 0
       dist = dist/norm(dist);
       %pont_to_cube_force = ( k_plane / dist_obs) * cross(robot_vel, cross(dist,robot_vel));
       line_normal = goal - robot_pos;
       line_normal = line_normal/norm(line_normal);
        
        
       dist = dist - (line_normal * dot(dist, line_normal));
       dist = dist/norm(dist);

       pont_to_cube_force = ( k_plane / dist_obs) * dist;
       force_direction = dist;
       cube_force_1 = pont_to_cube_force;
       

       robot_goal = goal - robot_pos;


       line1 = obs_point4 - obs_point2;
       line2 = obs_point4 - obs_point3;
        plane_n = cross(line1, line2);
        plane_n = plane_n/norm(plane_n);
        n_pi1 = DQ(plane_n);
        p_pi1 = DQ(obs_point1);
        d_pi1 = dot(p_pi1,n_pi1);
        
        % Calculate distance
        d_p_pi1 = (dot(DQ(robot_pos),n_pi1)-d_pi1);
        d_p_pi1 = d_p_pi1.q(1);
        %sig = sign(dot(robot_pos-obs_point1, plane_n));
        robot_obs = d_p_pi1 *plane_n;

       cos_theta = dot(-robot_obs, robot_goal) / (norm(robot_goal)*norm(robot_obs));

       obs_center_cube = [obs_center + [0 0 width];obs_center + [0 0 -width];obs_center + [0 width 0];obs_center + [0 -width 0];obs_center + [width 0 0];obs_center + [-width 0 0]];
       plane_direction = ["Z";"Z";"Y";"Y";"X";"X"];

       %points = PlanePoints(obs_center_cube(dist_index,:), width + 0.03, plane_direction(dist_index));
       points = PlanePoints(obs_center_cube(dist_index,:), width, plane_direction(dist_index));
       obs_point1 = points(1,:);
       obs_point2 = points(2,:);
       obs_point3 = points(3,:);
       obs_point4 = points(4,:);
       obs_point = [obs_point1; obs_point2;obs_point3;obs_point4];

       robot_intersect = norm(robot_obs)/cos_theta;
       intersect = robot_goal/norm(robot_goal) * robot_intersect + robot_pos;
       if min(obs_point(:,1)) -1.e-8  <= intersect(1) && intersect(1) <= max(obs_point(:,1)) + 1.e-8 ...
        && min(obs_point(:,2)) -1.e-8  <= intersect(2) && intersect(2) <= max(obs_point(:,2)) + 1.e-8...
        && min(obs_point(:,3)) -1.e-8  <= intersect(3) && intersect(3) <= max(obs_point(:,3)) + 1.e-8
      
           line1 = obs_point4 - obs_point2;
           line2 = obs_point4 - obs_point3;
           plane_n = cross(line1, line2);
           plane_n = plane_n/norm(plane_n);
           close_dist_ind = PTP_closet_line_inter_dist(goal, obs_point1, obs_point2, obs_point3, obs_point4);
           %close_dist = close_dist_ind(1:3);
           %close_dist = close_dist/norm(close_dist);
           close_ind = close_dist_ind(4);
           if close_ind < 1.1 && close_ind >0.8
                close_line = obs_point1 - obs_point4;
                edge_line = obs_point2 - obs_point1;
           elseif close_ind < 2.1 && close_ind >1.8
                close_line = obs_point2 - obs_point1;
                edge_line = obs_point3 - obs_point2;
           elseif close_ind < 3.1 && close_ind >2.8
                close_line = obs_point3 - obs_point2;
                edge_line = obs_point4 - obs_point3;
           elseif close_ind < 4.1 && close_ind >3.8
                close_line = obs_point4 - obs_point3;
                edge_line = obs_point1 - obs_point4;
           end
           close_line = close_line /norm(close_line);
           edge_line = edge_line/norm(edge_line);
           dire_line = cross(plane_n, edge_line);

          
           parallel_direction = dire_line;
           dire = sign(dot(close_line, parallel_direction));
           parallel_direction = parallel_direction * dire;

  
           %pont_to_cube_force = ( k_plane / dist_obs ) * cross(robot_vel, cross(parallel_direction,robot_vel));
           pont_to_cube_force = ( k_plane / dist_obs ) * parallel_direction

       end
    

    end
end