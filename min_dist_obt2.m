function [d, xo, yo] = min_dist_obt2(x1, y1, x2, y2, x, y)
% Calculate minimum distance between point (x,y) and axis-aligned rectangle
% defined by bottom-left (x1,y1) and top-right (x2,y2)

% Handle case where point is inside rectangle
if x >= x1 && x <= x2 && y >= y1 && y <= y2
    % Calculate distances to all sides
    left = x - x1;
    right = x2 - x;
    bottom = y - y1;
    top = y2 - y;
    
    % Find minimum distance to boundary
    [d, idx] = min([left, right, bottom, top]);
    
    % Set closest point on boundary
    switch idx
        case 1 % Left
            xo = x1; yo = y;
        case 2 % Right
            xo = x2; yo = y;
        case 3 % Bottom
            xo = x; yo = y1;
        case 4 % Top
            xo = x; yo = y2;
    end
    return;
end

% Calculate closest point for exterior cases
if x < x1
    x_closest = x1;
elseif x > x2
    x_closest = x2;
else
    x_closest = x;
end

if y < y1
    y_closest = y1;
elseif y > y2
    y_closest = y2;
else
    y_closest = y;
end

% Calculate distance and set output
d = norm([x - x_closest, y - y_closest]);
xo = x_closest;
yo = y_closest;
end