function plot_vf_line(path_start, path_end)
    line_start = csvread(path_start);
    line_end = csvread(path_end);
    for j=1:size(line_start,1)
        plot3([line_start(j,1), line_end(j,1)], [line_start(j,2), line_end(j,2)], [line_start(j,3), line_end(j,3)], '-k','LineWidth', 2);
        hold on
    end
end