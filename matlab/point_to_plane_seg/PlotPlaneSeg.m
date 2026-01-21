function PlotPlaneSeg(obs_point1, obs_point2, obs_point3, obs_point4)
    for j=1:size(obs_point1,1)

        % Define the four points of the plane
        x = [obs_point1(j,1), obs_point2(j,1), obs_point3(j,1), obs_point4(j,1)]; % X-coordinates of the corners
        y = [obs_point1(j,2), obs_point2(j,2), obs_point3(j,2), obs_point4(j,2)]; % Y-coordinates of the corners
        z = [obs_point1(j,3), obs_point2(j,3), obs_point3(j,3), obs_point4(j,3)]; % Z-coordinates of the corners
        
        % Plot the plane and fill with a dark color
        fill3(x, y, z, [0.1, 0.1, 0.1], 'FaceAlpha', 0.8); % Dark gray color, 80% opacity
        hold on;
        
        % Add edges for better visualization (optional)
        plot3(x, y, z, 'k-', 'LineWidth', 2);

 
        % %surf(X * obs_rad + obs_pos(j,1),Y * obs_rad + obs_pos(j,2), Z * obs_rad + obs_pos(j,3));
        % plot3([obs_point1(j,1), obs_point2(j,1)], [obs_point1(j,2), obs_point2(j,2)], [obs_point1(j,3), obs_point2(j,3)], "-");
        % hold on
    end
    
    % for j=1:size(obs_point2,1)
    %     %surf(X * obs_rad + obs_pos(j,1),Y * obs_rad + obs_pos(j,2), Z * obs_rad + obs_pos(j,3));
    %     plot3([obs_point3(j,1), obs_point2(j,1)], [obs_point3(j,2), obs_point2(j,2)], [obs_point3(j,3), obs_point2(j,3)], "-");
    %     hold on
    % end
    % 
    % for j=1:size(obs_point4,1)
    %     %surf(X * obs_rad + obs_pos(j,1),Y * obs_rad + obs_pos(j,2), Z * obs_rad + obs_pos(j,3));
    %     plot3([obs_point3(j,1), obs_point4(j,1)], [obs_point3(j,2), obs_point4(j,2)], [obs_point3(j,3), obs_point4(j,3)], "-");
    %     hold on
    % end
    % 
    % for j=1:size(obs_point1,1)
    %     %surf(X * obs_rad + obs_pos(j,1),Y * obs_rad + obs_pos(j,2), Z * obs_rad + obs_pos(j,3));
    %     plot3([obs_point1(j,1), obs_point4(j,1)], [obs_point1(j,2), obs_point4(j,2)], [obs_point1(j,3), obs_point4(j,3)], "-");
    %     hold on
    % end


    

   
end
