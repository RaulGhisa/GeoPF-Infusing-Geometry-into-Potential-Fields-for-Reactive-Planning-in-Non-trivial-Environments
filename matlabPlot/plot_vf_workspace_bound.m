function plot_vf_workspace_bound(workspaceboundary)
    [X,Y,Z] = sphere;
    obs_boundary = workspaceboundary(1:3);
    obs_boundary_rad = workspaceboundary(4);
    surf(X * obs_boundary_rad + obs_boundary(1),Y * obs_boundary_rad + obs_boundary(2), Z * obs_boundary_rad + obs_boundary(3),'FaceAlpha',.3,'EdgeColor','none');
end
