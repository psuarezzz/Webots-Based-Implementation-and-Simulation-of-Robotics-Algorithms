map = [0 0 0 0 0 1 0 0 0;
       0 1 1 1 0 0 0 1 0;
       0 0 0 1 1 1 1 1 1;
       1 1 0 1 0 0 0 1 0;
       0 0 0 1 0 1 0 1 0;
       0 1 1 1 0 1 0 0 0;
       0 0 0 0 0 1 0 1 0;
       1 1 1 1 1 1 1 1 0;
       0 0 0 0 0 0 0 0 0];

start_point = [1, 1];
goal_point = [9, 1];

max_iter = 20000;
step_size = 0.1;

tic

tree = [start_point, 0];  % [x, y, parent_index]

iterations_needed = inf;

% Main loop
goal_reached = false;
for iter = 1:max_iter
    % Generate a random point in map's boundaries
    random_point = [randi(size(map, 1)), randi(size(map, 2))];

    % Find the nearest point in the tree
    distances = sqrt(sum((tree(:, 1:2) - random_point).^2, 2)); %each of the tree's point distance to random point
    [~, nearest_index] = min(distances); %find the minimum
    nearest_point = tree(nearest_index, 1:2); %save the nearest point
    
    % Generate a new point towards the random point
    direction = random_point - nearest_point;
    direction = direction / norm(direction);    %calculate normalized direction
    new_point = nearest_point + step_size * direction;  %add step size on that direction
    
    % Check if the new point is within the map boundaries and not colliding
    if new_point(1) >= 1 && new_point(1) <= size(map, 1) && ... %check if it is on the map's boundaries
       new_point(2) >= 1 && new_point(2) <= size(map, 2) && ...
       map(round(new_point(1)), round(new_point(2))) == 0   %check if it collides with any wall
        % Add the new point and its index to the tree
        tree(end+1, :) = [new_point, nearest_index];
        
        % Check if the goal is reached
        if norm(new_point - goal_point) < step_size %distance to new point close enough to goal
            iterations_needed = iter;
            disp('Goal reached!');
            goal_reached = true;
            break;
        end
    end
end

time_elapsed = toc;
disp("Path searching time elapsed: " + time_elapsed);

disp("Iterations needed: " + iterations_needed);

% Reconstruct the path if goal is reached
if goal_reached
    path = [];
    current_index = size(tree, 1); %last point of the path's index is its tree's size
    while current_index ~= 0
        path = [tree(current_index, 1:2); path]; %add current index to the path list
        current_index = tree(current_index, 3); %get parent_index, to add next point to the tree
    end

    % Plot the map and the tree
    imshow(~map, 'InitialMagnification', 'fit'); % Invert map for visualization
    colormap(gray); % Set colormap to grayscale
    hold on;

    % Plot start and goal points
    plot(start_point(2), start_point(1), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
    plot(goal_point(2), goal_point(1), 'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g');

    % Plot the tree edges
    for i = 2:size(tree, 1)
        parent_index = tree(i, 3);
        parent_point = tree(parent_index, 1:2);
        current_point = tree(i, 1:2);
        plot([parent_point(2), current_point(2)], [parent_point(1), current_point(1)], 'm');
    end

    % Plot the reconstructed path
    plot(path(:, 2), path(:, 1), 'b', 'LineWidth', 2);
    title('RRT algorithm (Step = 1 m, 5000 iterations)')
    axis equal;
    grid on;
    hold off;
else
    disp('Error: Path not found within maximum iterations');
end

saveas(gcf, 'rrt.png');
