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

LIDAR_RANGE = 1;

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

lidar = wb_robot_get_device('lidar');
wb_lidar_enable(lidar, TIME_STEP);
wb_lidar_enable_point_cloud(lidar);


% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%


goal=[4.25, 1.25];


Kv = 5;
Kh = 1.5;

alpha = 0.1; % Attractive potential coefficient
beta = 0.1;  % Repulsive potential coefficient

step=0;

proximity_threshold = 0.1;

position_data = [];
speed = [];

while wb_robot_step(TIME_STEP) ~= -1
  
  step=step+1;
  
  pose = wb_gps_get_values(gps);
  xk = pose(1);
  yk = pose(2);
  
  position_data = [position_data; pose(1), pose(2)];
  
  angle = wb_compass_get_values(compass);
  theta_ = pi/2-atan2(angle(2), angle(1));
  
  range = wb_lidar_get_range_image(lidar);
  
  obstacle_pos = detect_obstacle(range, pose, theta_, LIDAR_RANGE);
  
  fprintf('Xk = %.2f, Yk = %.2f, Xo = %.2f, Yo = %.2f\n', pose(1), pose(2), obstacle_pos(1), obstacle_pos(2));
  
  attractive_term = attractive_field(pose(1:2), goal, alpha);
  
  repulsive_term = repulsive_field(pose(1:2), goal, obstacle_pos, beta);
  
  target_point = pose(1:2) + attractive_term + repulsive_term;
  
  thetak = atan2(target_point(2) - pose(2), target_point(1) - pose(1));
  
  error_speed = calculateDistance(pose, target_point);
  
  %control variables: v_ y gamma_
  v_ = Kv*error_speed;
  
  dif_angles = angdiff(theta_, thetak);

  gamma_ = Kh*dif_angles;
  
  speed(1) = v_ - gamma_;
  speed(2) = v_ + gamma_;
  
  if ((speed(2) > 6.25) || (speed(1) > 6.25))
    ratio = 6.15/max(speed);
    speed(2) = speed(2) * ratio;
    speed(1) = speed(1) * ratio;
  end
   
  apply_speed(left, speed(1));
  apply_speed(right, speed(2));
  
  
  % Verify if the robot is sufficiently close to the target
  if calculateDistance(pose, goal) < proximity_threshold
    
    apply_speed(left, 0.0);  % Stop left motor
    apply_speed(right, 0.0); % Stop right motor
    
    display('Target reached!')
    
    break; % Exit loop
  end
  
    
    %plot distance to target

%%%%%  
end

save('APF_data.mat', 'position_data')

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

function result = attractive_field(pose, goal, alpha)
        
    % Calculate attractive force
    inc_x_attract = -alpha * (pose(1) - goal(1));
    inc_y_attract = -alpha * (pose(2) - goal(2));
    
    result = [inc_x_attract, inc_y_attract];

end

function result = repulsive_field(pose, goal, obstacle_pos, beta)
    
    d_0 = 0.25;

    distance_to_obstacle = calculateDistance(pose, obstacle_pos);
    
    % Calculate repulsive force
    if distance_to_obstacle < d_0
      repulsive_force = (1 / distance_to_obstacle - 1 / d_0) * (1 / (distance_to_obstacle^2)) * (1 / (2 * distance_to_obstacle)) * (obstacle_pos - pose);
    else
      repulsive_force = [0, 0];
    end
       
    result = -beta * repulsive_force;
    
end

function obstacle_coordinates = detect_obstacle(lidar_data, robot_position, robot_angle, max_distance)
    % Define LiDAR's scan angle
    scan_angle = 180;
    
    % LiDAR resolution
    num_points = 512;
    
    angle_increment = scan_angle / (num_points - 1);
    
    % Closest point
    [min_distance, min_index] = min(lidar_data);

    % Relative angle robot-obstacle
    obstacle_angle = robot_angle - (scan_angle / 2) + (min_index - 1) * angle_increment;
    

    obstacle_angle_rad = deg2rad(obstacle_angle);
    
    % Calculate relative obstacle coordinates
    obstacle_relative_x = min_distance * cos(obstacle_angle_rad);
    obstacle_relative_y = min_distance * sin(obstacle_angle_rad);
    
    % Calculate absolute coordinates
    obstacle_coordinates = [robot_position(1) + obstacle_relative_x, robot_position(2) + obstacle_relative_y];
    
    if min_distance > max_distance
        obstacle_coordinates = [inf, inf]; % Obstacle is out of LiDAR's range
    end
end

    
% cleanup code goes here: write data to files, etc.
