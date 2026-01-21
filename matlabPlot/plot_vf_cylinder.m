function plot_vf_cylinder(path_start, path_end, path_rad)
    % Load data
    cylinder_start = csvread(path_start); % Nx3
    cylinder_end = csvread(path_end);     % Nx3
    cylinder_rad = csvread(path_rad);     % Nx1

    num_cylinders = size(cylinder_start, 1);

    % Base unit cylinder aligned with z-axis from 0 to 1
    [Xc0, Yc0, Zc0] = cylinder(1, 30);  % 30 segments for roundness

    for i = 1:num_cylinders
        P1 = cylinder_start(i, :).';
        P2 = cylinder_end(i, :).';
        radius = cylinder_rad(i);

        % Height and direction
        direction = P2 - P1;
        height = norm(direction);
        dir_unit = direction / height;

        % Scale base cylinder to this radius and height
        Xc = radius * Xc0;
        Yc = radius * Yc0;
        Zc = height * Zc0;

        % Flatten the cylinder mesh into vectors
        [nRows, nCols] = size(Xc);
        pts = [Xc(:), Yc(:), Zc(:)]';  % 3 x N

        % Rotation matrix to align [0; 0; 1] to dir_unit
        z_axis = [0; 0; 1];
        if (dot(z_axis, dir_unit)) > 0.999 
            R = eye(3);
        elseif (dot(z_axis, dir_unit)) < -0.999 
            R = -eye(3);
        else
            v = cross(z_axis, dir_unit);
            s = norm(v);
            c = dot(z_axis, dir_unit);
            vx = [  0   -v(3)  v(2);
                   v(3)   0  -v(1);
                  -v(2) v(1)   0];
            R = eye(3) + vx + vx^2 * ((1 - c) / s^2);
        end

        % Rotate and translate
        rotated_pts = R * pts;
        rotated_pts = rotated_pts + P1;

        % Reshape back for surf
        Xc_plot = reshape(rotated_pts(1, :), nRows, nCols);
        Yc_plot = reshape(rotated_pts(2, :), nRows, nCols);
        Zc_plot = reshape(rotated_pts(3, :), nRows, nCols);

        % Plot
        obj = surf(Xc_plot, Yc_plot, Zc_plot, ...
            'FaceColor', 'k', 'EdgeColor', 'none', 'FaceAlpha', 0.5);
        dddd = P2-P1;
        % dx = dddd(1);
        % dy = dddd(2);
        % dz = dddd(3);
        % obj.XData = obj.XData + dx;
        % obj.YData = obj.YData + dy;
        % obj.ZData = obj.ZData + dz;



        hold on;
    end

    axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3); grid off;
end
