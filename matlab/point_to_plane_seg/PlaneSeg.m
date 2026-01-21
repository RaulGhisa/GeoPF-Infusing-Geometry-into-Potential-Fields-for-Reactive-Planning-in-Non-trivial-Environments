function plane = PlaneSeg(obs_point1, obs_point2,obs_point3,obs_point4)
    obs_line1 = DQ(obs_point1 - obs_point2);
    obs_line2 = DQ(obs_point3 - obs_point2);
    obs_direction = cross(obs_line1, obs_line2);
    obs_direction = obs_direction/norm(obs_direction);
    n_pi1 = DQ(obs_direction);
    n_pi1 = n_pi1/norm(n_pi1);
    p_pi1 = DQ(obs_point1);
    d_pi1 = dot(p_pi1, n_pi1);
    plane = n_pi1 + DQ.E*d_pi1;
end
