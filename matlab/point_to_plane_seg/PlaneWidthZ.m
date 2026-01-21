function widths = PlaneWidthZ(width)
    width1 = [width(1) width(1) 0];
    width2 = [-width(1) width(1) 0];
    width3 = [-width(1) -width(1) 0];
    width4 = [width(1) -width(1) 0];
    widths = [width1;width2;width3;width4];
end
