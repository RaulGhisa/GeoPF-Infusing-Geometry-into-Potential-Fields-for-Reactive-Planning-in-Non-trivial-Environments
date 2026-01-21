function widths = PlaneWidthY(width)
    width1 = [width 0 width];
    width2 = [-width 0 width];
    width3 = [-width 0 -width];
    width4 = [width 0 -width];
    widths = [width1;width2;width3;width4];
end
