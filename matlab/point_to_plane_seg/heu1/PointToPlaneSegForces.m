
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function pont_to_planeSeg_force = PointToPlaneSegForces(obs_center, width, plane_direction, robot_pos, robot_vel, goal, k_plane,plane_dist, k_line)

%width = width + 0.03;
points = PlanePoints(obs_center, width, plane_direction);
obs_point1 = points(1,:);
obs_point2 = points(2,:);
obs_point3 = points(3,:);
obs_point4 = points(4,:);
obs_point = [obs_point1; obs_point2;obs_point3;obs_point4];
%if plane_direction == "Y"
%    obs_point1 = obs_point1 + [0 0 3];
%    obs_point2 = obs_point2 + [0 0 3];
%end


% Function that computes the circular forces
dist_obs = norm(PTPdist_pos_obs(robot_pos, obs_point1, obs_point2, obs_point3, obs_point4));
pont_to_planeSeg_force = zeros(1,3);

%if dist_obs < 0.001
%    width = width + 0.03;
%    if plane_direction == "Y"
%        obs_center = obs_center + [0 -0.025 0];
%    end
    
%    points = PlanePoints(obs_center, width, plane_direction);
%    obs_point1 = points(1,:);
%    obs_point2 = points(2,:);
%    obs_point3 = points(3,:);
%    obs_point4 = points(4,:);
%    obs_point = [obs_point1; obs_point2;obs_point3;obs_point4];
%    if plane_direction == "Y"
%        obs_point1 = obs_point1 + [0 0 3];
%        obs_point2 = obs_point2 + [0 0 3];
%    end
    
    
%    % Function that computes the circular forces
%    dist_obs = norm(PTPdist_pos_obs(robot_pos, obs_point1, obs_point2, obs_point3, obs_point4));
%    pont_to_planeSeg_force = zeros(1,3);
%end


%center_of_Plane = (obs_point4 + obs_point3 + obs_point2 + obs_point1)/4;
%dist_obs_curr = norm(robot_pos - center_of_Plane) - 0.05;
% define how much distance should consider force
if dist_obs < 0.1
   if norm(robot_vel) ~= 0
        


       dist = PointToPlanecomp_dist(robot_pos, obs_point1, obs_point2, obs_point3, obs_point4, goal);
       dist = dist/norm(dist);
      
       % pont_to_planeSeg_force = ( k_plane / dist_obs) * cross(robot_vel, cross(dist,robot_vel));
       pont_to_planeSeg_force = ( k_plane / dist_obs) * dist;



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
           
        % robot_obs = PTPdist_pos_obs(robot_pos, obs_point1, obs_point2, obs_point3, obs_point4);
        % robot_obs_direct = robot_obs/norm(robot_obs);

       robot_goal = goal - robot_pos;
       
       cos_theta = dot(-robot_obs, robot_goal) / (norm(robot_goal)*norm(robot_obs));
       robot_intersect = norm(robot_obs)/cos_theta;
       intersect = robot_goal/norm(robot_goal) * robot_intersect + robot_pos;
       if min(obs_point(:,1)) -1.e-8  <= intersect(1) && intersect(1) <= max(obs_point(:,1)) + 1.e-8 ...
        && min(obs_point(:,2)) -1.e-8  <= intersect(2) && intersect(2) <= max(obs_point(:,2)) + 1.e-8...
        && min(obs_point(:,3)) -1.e-8  <= intersect(3) && intersect(3) <= max(obs_point(:,3)) + 1.e-8

            
            %obs_center =obs_center +0.2 * robot_obs;

            points = PlanePoints(obs_center, width, plane_direction);
            obs_point1 = points(1,:);
            obs_point2 = points(2,:);
            obs_point3 = points(3,:);
            obs_point4 = points(4,:);
      
           line1 = obs_point4 - obs_point2;
           line2 = obs_point4 - obs_point3;
           plane_n = cross(line1, line2);
           plane_n = plane_n/norm(plane_n);
           close_dist_ind = PTP_closet_line_inter_dist(robot_pos, obs_point1, obs_point2, obs_point3, obs_point4);
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
           elseif close_ind dist_obs< 4.1 && close_ind >3.8
                close_line = obs_point4 - obs_point3;
                edge_line = obs_point1 - obs_point4;
           end
           close_line = close_line /norm(close_line);
           edge_line = edge_line/norm(edge_line);
           dire_line = cross(plane_n, edge_line);

          
            parallel_direction = dire_line;
            dire = sign(dot(close_line, parallel_direction));
            parallel_direction = parallel_direction * dire;

           %pont_to_planeSeg_force = ( k_plane / dist_obs ) * cross(robot_vel, cross(parallel_direction,robot_vel));
           pont_to_planeSeg_force = ( k_plane / dist_obs ) * parallel_direction;%- att_force;

           dist_obs

       
       end
   end
end