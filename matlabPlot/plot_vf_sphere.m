function plot_vf_sphere(path_circle, path_rad)
    [X,Y,Z] = sphere;
    circles = csvread(path_circle);
    circlesrad = csvread(path_rad);
    for j = 1:size(circles,1)
        h = surf(X * circlesrad(j) + circles(j,1), ...
                 Y * circlesrad(j) + circles(j,2), ...
                 Z * circlesrad(j) + circles(j,3));
        set(h, 'FaceColor', [0.5 0.5 0.5], ...   % Gray color
               'EdgeColor', 'none', ...          % No wireframe edges
               'FaceAlpha', 0.3);                % Transparency (0 = invisible, 1 = opaque)
        hold on
    end
end