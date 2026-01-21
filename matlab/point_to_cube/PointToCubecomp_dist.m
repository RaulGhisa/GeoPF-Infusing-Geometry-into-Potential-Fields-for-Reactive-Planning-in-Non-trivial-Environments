function dist_and_index = PointToCubecomp_dist(robot_pos, obs_center, width, goal)

cube = CubeSeg(obs_center,width);  
robot_obs = zeros(6,3);
robot_obs_norm = zeros(1,6); 
for i=1:size(cube,1)
    obs_point1 = cube(i,1:3);
    obs_point2 = cube(i,4:6);
    obs_point3 = cube(i,7:9);
    obs_point4 = cube(i,10:12);
    robot_obs(i,:) = PTPdist_pos_obs(robot_pos, obs_point1, obs_point2, obs_point3, obs_point4);
    robot_obs_norm(i) = norm(robot_obs(i,:));
    if i==3
        obs_point1 = obs_point1;
        obs_point2 = obs_point2;
        obs_point3 = obs_point3;
        obs_point4 = obs_point4;
    end

end

robot_obs_norm = robot_obs_norm;
robot_obs = robot_obs;
dist_index = find(robot_obs_norm == min(robot_obs_norm));


robot_obs_min = robot_obs(dist_index(1),:);

%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
line_normal = goal - robot_pos;
line_normal = line_normal/norm(line_normal);


dist =robot_obs_min;% robot_obs_min - (line_normal * dot(robot_obs_min, line_normal));

if norm(dist) < 1e-10
    dist = -1 + 2 * rand(size(obs_point1));
end

dist_and_index = zeros(1,7);

dist_and_index(1,1:3) = dist;
dist_and_index(1,4) = dist_index(1);
dist_and_index(1,5:7) = robot_obs_min;

