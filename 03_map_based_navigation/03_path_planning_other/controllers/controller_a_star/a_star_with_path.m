function [end_node, shortest_distances, path, error, path_cost] = a_star_with_path(adjacency_matrix, start_node, end_node, heuristic_type)
    num_nodes = size(adjacency_matrix, 1);
  
    heuristic = zeros(1, num_nodes);
    
    % Define the heuristic based on the specified type
    
    if strcmp(heuristic_type, 'Euclidean')
        % Euclidean distance heuristic
        [end_row, end_col] = ind2sub(size(adjacency_matrix), end_node);
        for i = 1:num_nodes
            [row, col] = ind2sub(size(adjacency_matrix), i);
            heuristic(i) = sqrt((row - end_row)^2 + (col - end_col)^2);
        end
    
    elseif strcmp(heuristic_type, 'Manhattan')
        % Manhattan distance heuristic
        [end_row, end_col] = ind2sub(size(adjacency_matrix), end_node);
        for i = 1:num_nodes
            [row, col] = ind2sub(size(adjacency_matrix), i);
            heuristic(i) = abs(row - end_row) + abs(col - end_col);
        end
    
    elseif strcmp(heuristic_type, 'Chebyshev')
    % Chebyshev distance heuristic
    [end_row, end_col] = ind2sub(size(adjacency_matrix), end_node);
    for i = 1:num_nodes
        [row, col] = ind2sub(size(adjacency_matrix), i);
        dx = abs(row - end_row);
        dy = abs(col - end_col);
        heuristic(i) = max(dx, dy);
    end

    elseif strcmp(heuristic_type, 'Octile')
    % Octile distance heuristic
    [end_row, end_col] = ind2sub(size(adjacency_matrix), end_node);
    for i = 1:num_nodes
        [row, col] = ind2sub(size(adjacency_matrix), i);
        dx = abs(row - end_row);
        dy = abs(col - end_col);
        heuristic(i) = max(dx, dy) + (sqrt(2) - 1) * min(dx, dy);
    end

    elseif strcmp(heuristic_type, 'None')
        heuristic = zeros(1, num_nodes);

    else
        error('Invalid heuristic type. Please choose "Euclidean" or "Manhattan".');
    end

    % Initialize arrays
    visited = false(1, num_nodes);
    shortest_distances = inf(1, num_nodes);
    prev = zeros(1, num_nodes);
    
    % Set distance of start node to 0
    shortest_distances(start_node) = 0;
    prev(start_node) = start_node;  % Initialize prev for the start node
    
    % Main loop
    while any(~visited)
        % Find the unvisited node with the lowest estimated total cost
        unvisited_nodes = find(~visited);
        total_cost = shortest_distances + heuristic;
        [~, current_node] = min(total_cost(unvisited_nodes));
        current_node = unvisited_nodes(current_node);
        
        % Mark current node as visited
        visited(current_node) = true;
        
        % Update distances to neighbors
        neighbors = find(adjacency_matrix(current_node, :) ~= inf);
        for neighbor = neighbors
            if ~visited(neighbor)
                new_distance = shortest_distances(current_node) + adjacency_matrix(current_node, neighbor);
                if new_distance < shortest_distances(neighbor)
                    shortest_distances(neighbor) = new_distance;
                    prev(neighbor) = current_node;
                end
            end
        end
    end
    
    % Reconstruct path to the end node and calculate the path cost
    if shortest_distances(end_node) == inf && all(adjacency_matrix(end_node, :) == inf)
        path = []; % No viable path found, unreachable node
        error = true;
        path_cost = inf;
    elseif shortest_distances(end_node) == inf
        path = []; % No viable path found, but node is reachable
        error = false;
        path_cost = inf;
    else
        path = end_node;
        current_node = end_node;
        path_cost = 0;
        while current_node ~= start_node
            prev_node = prev(current_node);
            path_cost = path_cost + adjacency_matrix(prev_node, current_node);
            current_node = prev_node;
            path = [current_node path];
        end
        error = false;
    end
end

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


