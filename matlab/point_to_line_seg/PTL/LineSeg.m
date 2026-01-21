function line = LineSeg(obs_start, obs_end)
    l1= DQ(obs_start - obs_end);
    p_l1 = obs_start;
    m1 = cross(p_l1,l1);
    line = l1 + E_*m1;
end
