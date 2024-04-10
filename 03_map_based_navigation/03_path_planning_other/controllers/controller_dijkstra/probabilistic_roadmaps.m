map = [0 1 0 0 1 0 0 1 1;
       0 0 0 0 1 1 0 0 0;
       0 1 0 0 0 1 0 1 0;
       0 1 1 0 0 0 0 0 0;
       0 1 0 0 1 1 0 0 1;
       0 0 0 0 0 0 0 1 1;
       1 1 0 0 1 1 0 0 0;
       1 0 0 0 0 1 0 1 0;
       0 0 0 1 0 0 0 1 0];

n_points = 2000;
d0=0.6;

start_node = [0.5, 0.5];
end_node = [8.5, 8.5];

[rows, cols] = size(map);

G=[];

for i=1:n_points
    % Generate a random point on the maps bounds
    random_point = [rand*rows, rand*cols];
    
    % Check in which of the map cells the point has been generated
    x = ceil(random_point(1)); % Columna
    y = ceil(random_point(2)); % Fila

    if map(x,y)==0
        G=[G; random_point];
    end
    
end

% Add start and end points
G = [start_node; G];
G = [G; end_node];

%Initialize links and distances matrix
q = ones(size(G, 1)) * inf;
for i=1:size(G, 1)
    q(i,i)=0;
end

% Check all pairs of points
for i = 1:size(G, 1)
    for j = i+1:size(G, 1)
        % Obtain coordinates of current point and the next point
        x1 = G(i, 1);
        y1 = G(i, 2);
        x2 = G(j, 1);
        y2 = G(j, 2);

        % Calculate distance between the points
        distance = calculateDistance([x1, y1], [x2, y2]);

        % If the distance is less than or equal to d0
        if distance <= d0
            % Calculate Bresenham line
            [x_line, y_line] = bresenham(x1, y1, x2, y2, map);

            % Check if all points in the line are free in the map
            possible = all(map(sub2ind(size(map), x_line, y_line)) == 0);

            if possible
                % Update adjacency matrix with distance
                q(i, j) = distance;
                q(j, i) = distance;
            end
        end
    end
end

clf(figure(1));

plot_graph(G, q);

save('map_data.mat', "map", "q", "G");

function [path_cells_x, path_cells_y] = bresenham(x1, y1, x2, y2, map)
    % Round the points to the nearest integers
    x1 = ceil(x1); y1 = ceil(y1);
    x2 = ceil(x2); y2 = ceil(y2);
    
    % Calculate differences and determine steepness
    dx = abs(x2 - x1);
    dy = abs(y2 - y1);
    steep = dy > dx;

    % Swap coordinates if the line is steep
    if steep
        temp = x1;
        x1 = y1;
        y1 = temp;
        
        temp = x2;
        x2 = y2;
        y2 = temp;
    end

    % Ensure x1 is less than x2
    if x1 > x2
        temp = x1;
        x1 = x2;
        x2 = temp;

        temp = y1;
        y1 = y2;
        y2 = temp;
    end

    % Calculate differentials
    dx = x2 - x1;
    dy = abs(y2 - y1);
    
    % Initialize error and steps
    error = dx / 2;
    ystep = (y1 < y2) - (y1 > y2);
    y = y1;

    % Initialize path cells
    path_cells_x = [];
    path_cells_y = [];

    % Iterate over x values and determine corresponding y values
    for x = x1:x2
        % Add current cell to the path
        if steep
            path_cells_x = [path_cells_x; y];
            path_cells_y = [path_cells_y; x];
        else
            path_cells_x = [path_cells_x; x];
            path_cells_y = [path_cells_y; y];
        end

        % Update error and y
        error = error - dy;
        if error < 0
            y = y + ystep;
            error = error + dx;
        end
    end

    % Ensure path is within map boundaries
    path_cells_x(path_cells_x < 1) = 1;
    path_cells_x(path_cells_x > size(map, 2)) = size(map, 2);
    path_cells_y(path_cells_y < 1) = 1;
    path_cells_y(path_cells_y > size(map, 1)) = size(map, 1);
end

function plot_graph(G, q)

    scatter(G(:, 1), G(:, 2), 'filled');
    hold on;

    num_nodos = size(G, 1);

    for i = 1:num_nodos
        for j = i+1:num_nodos
            if q(i, j) ~= inf
                plot([G(i, 1), G(j, 1)], [G(i, 2), G(j, 2)], 'b');
            end
        end
    end
    
    xlim([min(G(:, 1)) - 1, max(G(:, 1)) + 1]);
    ylim([min(G(:, 2)) - 1, max(G(:, 2)) + 1]);
    
    
    grid on;
    
    hold off;
end

function dist = calculateDistance(pi, pf)
  dx = pf(1)-pi(1);
  dy = pf(2)-pi(2);

  dist = sqrt(dx^2 + dy^2);
end