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

%Change start and end nodes in probabilistic_roadmaps.m
start_point = [0.5, 0.5];
end_point = [8.5, 8.5];


load('map_data.mat');

start_node = 1;
end_node = size(G, 1);

%heuristic_method = 'Euclidean';
%heuristic_method = 'Manhattan';
%heuristic_method = 'Chebyshev';
%heuristic_method = 'Octile';
heuristic_method = 'None'; %makes A* algorithm to work as Dijkstra's


tic
% Find the shortest path
[end_node, shortest_distances, path, error, path_cost] = a_star_with_path(q, start_node, end_node, heuristic_method);
time_elapsed = toc;

disp("Heuristic method: " + heuristic_method);

disp("Path searching time elapsed: " + time_elapsed);

if error
    disp("No viable path to node " + end_node);
elseif shortest_distances(end_node) == inf
    disp("No viable path to node " + end_node);
else
    disp("Shortest path to node " + end_node + ":");
    for i = 1:length(path)
        fprintf("%d, ", path(i));
    end
    fprintf("\n")
    disp("Path cost: " + path_cost);
end

trajectory = G(path, :)/2;



% Plot the trajectory
clf(figure(2));
figure(2);
plot(trajectory(:, 1), trajectory(:, 2), '-o', 'LineWidth', 2, 'MarkerSize', 8);
xlabel('X axis (m)');
ylabel('Y axis (m)');
title('Planned Path - A* (None)');
grid on;
    

Kv = 5;
Kh = 0.9;

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
  

end

saveas(gcf, 'a_star.png');

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
  
    threshold = 0.07; %robot's diameter
    
    
    if (calculateDistance(pose, trajectory(pursuit_step, :)) < threshold) && (size(trajectory,1) > pursuit_step)
      pursuit_step = pursuit_step + 1;
    end
    
    
    direction_vector=[trajectory(pursuit_step, 1)-pose(1), trajectory(pursuit_step, 2)-pose(2)];
    
    magnitude = sqrt(direction_vector(1)^2 + direction_vector(2)^2);
    
    normalized_direction = [direction_vector(1)/magnitude, direction_vector(2)/magnitude];
    
    target_point = [pose(1)+L*normalized_direction(1), pose(2)+L*normalized_direction(2)];
    
end
