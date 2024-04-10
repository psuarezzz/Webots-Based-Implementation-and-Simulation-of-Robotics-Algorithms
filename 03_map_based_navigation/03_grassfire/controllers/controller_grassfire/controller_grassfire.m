% MATLAB controller for Webots
% File: controller_vete_punto.m
% Date:
% Description:
% Author:
% Modifications:

function my_controller

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

TIME_STEP = 64;
CAMERA_TIME_STEP=256;

TILE_SIZE = 0.2;

%get and enable device:
%camera = wb_robot_get_device('camera');
%wb_camera_enable(camera, CAMERA_TIME_STEP);

left = wb_robot_get_device('left wheel motor');
right = wb_robot_get_device('right wheel motor');

wb_motor_set_position(left, inf);
wb_motor_set_position(right, inf);
wb_motor_set_velocity(left, 0);
wb_motor_set_velocity(right, 0);


gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);


compass = wb_robot_get_device('compass');
wb_compass_enable(compass, TIME_STEP);

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%

    map = [[0 0 0 0 0 1 0 0 0];
           [0 1 1 1 0 0 0 1 0];
           [0 0 0 1 1 1 1 1 1];
           [1 1 0 1 0 0 0 1 0];
           [0 0 0 1 0 1 0 1 0];
           [0 1 1 1 0 1 0 0 0];
           [0 0 0 0 0 1 0 1 0];
           [1 1 1 1 1 1 1 1 0];
           [0 0 0 0 0 0 0 0 0]];

    start = [1, 1];
    goal = [9, 1];
    
    tic;
    path = grassfire(map, start, goal);
    time_to_calculate_path = toc;
    
    fprintf('Time taken to calculate the path: %.6f s\n', time_to_calculate_path);
    
    % Display the path points
    %disp('Path Points:');
    %disp(path);
    
    % Plot the maze and the path
    figure;
    maze = binaryOccupancyMap(map);
    show(maze)
    hold on;
    plot(path(:, 2) - 0.5, size(map, 1) - path(:, 1) + 1 - 0.5, 'g', 'LineWidth', 1); % Adjusted plotting with 0.5 offset
    title('Maze and path planned')
    xlabel('X Coordinate (m)');
    ylabel('Y Coordinate (m)');
    hold off;
    
    saveas(gcf, 'maze.png');
    close all;
    
    trajectory = indicesToGPS(path, TILE_SIZE);

Kv = 5;
Kh = 1;

%look ahead distance
L=0.1;

goal_point=trajectory(end, :);

step=0;
pursuit_step=2;

proximity_threshold = 0.01;

position_data=[];
speed=[];

while wb_robot_step(TIME_STEP) ~= -1
  
  step=step+1;
  
  pose = wb_gps_get_values(gps);
  xk = pose(1);
  yk = pose(2);
  
  position_data=[position_data; xk, yk];
  
  [target_point, pursuit_step] = purePursuit2(pursuit_step, pose, trajectory, L);
  
  angle = wb_compass_get_values(compass);
  theta_ = pi/2-atan2(angle(2), angle(1));
  
  thetak = atan2(target_point(2) - pose(2), target_point(1) - pose(1));
  
  error_speed=sqrt((target_point(1)-pose(1))^2+(target_point(2)-pose(2))^2);
  
  %control variables: v_ y gamma_
  v_ = Kv*error_speed;
  
  dif_angles = angdiff(theta_, thetak);

  gamma_ = Kh*dif_angles;
       
  speed(1) = v_;
  speed(2) = v_;
  
  speed(1) = speed(1) - gamma_;
  speed(2) = speed(2) + gamma_;
 
  apply_speed(left, speed(1));
  apply_speed(right, speed(2));
  
  
  % Verify if the robot is sufficiently close to the target
  if calculateDistance(pose, goal_point) < proximity_threshold
    
    apply_speed(left, 0.0);  % Stop left motor
    apply_speed(right, 0.0); % Stop right motor
    
    display('Target reached!')
    
    break; % Exit loop
  end
  
   
%%%%%  
end

wb_robot_cleanup();
wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);

end

function dist = calculateDistance(pi, pf)
  dx = pf(1)-pi(1);
  dy = pf(2)-pi(2);

  dist = sqrt(dx^2 + dy^2);
end

function apply_speed(motor, velocidad)
    
  MOTOR_MAX_SPEED = 6.28; %Motor max velocity

  if velocidad > MOTOR_MAX_SPEED
    velocidad = MOTOR_MAX_SPEED;
  elseif velocidad < -MOTOR_MAX_SPEED
    velocidad = -MOTOR_MAX_SPEED;
  end
  
  wb_motor_set_velocity(motor, velocidad);
end

function [target_point, pursuit_step] = purePursuit2(pursuit_step, pose, trajectory, L)
  
    threshold = 0.02;
    
    
    if (calculateDistance(pose, trajectory(pursuit_step, :)) < threshold) && (size(trajectory,1) > pursuit_step)
      pursuit_step = pursuit_step + 1;
    end
    
    
    direction_vector=[trajectory(pursuit_step, 1)-pose(1), trajectory(pursuit_step, 2)-pose(2)];
    
    magnitude = sqrt(direction_vector(1)^2 + direction_vector(2)^2);
    
    normalized_direction = [direction_vector(1)/magnitude, direction_vector(2)/magnitude];
    
    target_point = [pose(1)+L*normalized_direction(1), pose(2)+L*normalized_direction(2)];
end

function path = grassfire(map, start, goal)
    [rows, cols] = size(map);
    visited = false(rows, cols);
    queue = start;
    parent = zeros(rows, cols, 2);
    
    while ~isempty(queue)
        current = queue(1,:);
        queue(1,:) = [];
        
        if isequal(current, goal)
            path = reconstructPath(parent, start, goal);
            return;
        end
        
        visited(current(1), current(2)) = true;
        
        neighbors = getNeighbors(map, current);
        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i,:);
            if ~visited(neighbor(1), neighbor(2))
                queue = [queue; neighbor];
                visited(neighbor(1), neighbor(2)) = true;
                parent(neighbor(1), neighbor(2), :) = current;
            end
        end
    end
    
    error("Goal is unreachable");
end

function path = reconstructPath(parent, start, goal)
    path = goal;
    current = goal;
    
    while ~isequal(current, start)
        current = squeeze(parent(current(1), current(2), :))';
        path = [current; path];
    end
end

function neighbors = getNeighbors(map, cell)
    [rows, cols] = size(map);
    r = cell(1);
    c = cell(2);
    
    neighbors = [];
    
    if r > 1 && map(r-1,c) == 0
        neighbors = [neighbors; [r-1, c]];
    end
    if r < rows && map(r+1,c) == 0
        neighbors = [neighbors; [r+1, c]];
    end
    if c > 1 && map(r,c-1) == 0
        neighbors = [neighbors; [r, c-1]];
    end
    if c < cols && map(r,c+1) == 0
        neighbors = [neighbors; [r, c+1]];
    end
end

function gps_coords = indicesToGPS(path, tile_size)
    % path: matrix containing the row and column indices of the path points
    % tile_size: size of each tile in meters

    % Define the center of cell (5,5) as the origin in meters
    origin_meters = [-(5 - 1/2) * tile_size, (5 - 1/2) * tile_size];

    % Calculate the GPS coordinates of each point in the path
    gps_coords = zeros(size(path, 1), 2);
    for i = 1:size(path, 1)
        % Calculate the distance in meters from the origin to the current point
        distance = [(path(i, 2) - 1/2) * tile_size, -(path(i, 1) - 1/2) * tile_size];

        % Convert the distance to GPS coordinates using the origin
        gps_coords(i, 1) = origin_meters(1) + distance(1);
        gps_coords(i, 2) = origin_meters(2) + distance(2);
    end
end

    
% cleanup code goes here: write data to files, etc.
