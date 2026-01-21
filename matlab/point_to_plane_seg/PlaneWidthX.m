function widths = PlaneWidthX(width)
    width1 = [0 width(1) width(1)];
    width2 = [0 -width(1) width(1)];
    width3 = [0 -width(1) -width(1)];
    width4 = [0 width(1) -width(1)];
    widths = [width1;width2;width3;width4];
end
