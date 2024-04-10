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
CAMERA_TIME_STEP = 256;

MAX_DIST = 0.20;

%set this parameter as true to enable and view camera (reduces performance)
get_camera = false;

% Wall follow controller parameters

K_follow = 7; %Proportional gain

% Advance controller parameters

K_advance = 5; % Proportional gain

% Turn type
%turn = 'left';
turn = 'right';

%Initial state

state = "align";

goal = [2.98, 0.72]; % Goal point

proximity_threshold = 0.01;

position_data = [];
time_data = [];
distance_to_target = [];
lidar_data = [];


%get and enable devices, e.g.:
if get_camera==true
  camera = wb_robot_get_device('camera');
  wb_camera_enable(camera, CAMERA_TIME_STEP);
end;


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
pose = wb_gps_get_values(gps);

lidar = wb_robot_get_device('lidar');
wb_lidar_enable(lidar, TIME_STEP);
wb_lidar_enable_point_cloud(lidar);

log_file = fopen('log.txt', 'w');
if log_file == -1
    error('Could not open the log file.');
end

step = 0;

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%

fprintf("Bug2 algorithm starting...\n");
fprintf("Initial state is: %s\n", state);

fprintf(log_file, "Bug2 algorithm starting...\n");
fprintf(log_file, "Initial state is: %s\n", state);


while wb_robot_step(TIME_STEP) ~= -1
  
  step = step + 1;
  
  pose = wb_gps_get_values(gps);
  
  time_data=[time_data, wb_robot_get_time()];
  
  position_data=[position_data; pose(1), pose(2)];
  
  angle = wb_compass_get_values(compass);
  
  theta = pi/2-atan2(angle(2), angle(1));
  
  range = wb_lidar_get_range_image(lidar);
  
  lidar_data = [lidar_data; range];
  
  if step == 1
    m = (goal(2)-pose(2))/(goal(1)-pose(1));
    n = pose(2);
    
  end
  
  distance_to_target(step) = calculateDistance(pose(1), pose(2), goal(1), goal(2));
  
  if distance_to_target(step) < proximity_threshold
    
    apply_speed(left, 0);
    apply_speed(right, 0);
    
    fprintf("Target reached!\n");    
    fprintf(log_file, "Target reached!\n");
    
    break;
  end;
    
  
  
  if state == "align"
    
    is_aligned = align_to_goal(theta, pose, goal, left, right);
    
    if is_aligned == true
      state = "advance";
      fprintf("State changed to: %s\n", state);
      fprintf(log_file,"State changed to: %s\n", state);
    end
    
  end
  
  if state == "advance"  
    
    apply_speed(left, K_advance*distance_to_target(step))
    apply_speed(right, K_advance*distance_to_target(step))
    
  end
  
  if state == "advance"
    for i = 1:length(range)
      if range(i) < MAX_DIST
          
          fprintf('Wall detected!\n', range(i));
          fprintf(log_file, 'Wall detected!\n', range(i));
          
          
          [stop_distance, min_index] = min(range(1:512));
          
          apply_speed(left, 0)
          apply_speed(right, 0)
          
          hit_point_dist = calculateDistance(pose(1), pose(2), goal(1), goal(2));
          
          state = "rotate";
          fprintf("State changed to: %s\n", state)
          fprintf(log_file,"State changed to: %s\n", state);
  
  
          break;
          
      end
    end
  end
  
  if state == "rotate"     
        
        if strcmp(turn, 'right')
          
          apply_speed(left, 0.8)
          apply_speed(right, -0.8)
          wall_error = range(1) - stop_distance;
        
        elseif strcmp(turn, 'left')
          
          apply_speed(left, -0.8)
          apply_speed(right, 0.8)
          wall_error = range(512) - stop_distance;
        end
          
        
        if abs(wall_error)<0.0012 
          
          apply_speed(left, 0)
          apply_speed(right, 0)
          
          start_point = pose;
          
          state = "follow_wall";
          fprintf("State changed to: %s\n", state)
          fprintf(log_file,"State changed to: %s\n", state);
          
          
        end

  end
  
  
  if state == "follow_wall"
        
        [~, min_index] = min(range(1:512));
        
        error = range(min_index) - MAX_DIST;
        
        control = K_follow * error;
        
        if strcmp(turn, 'right')
          apply_speed(left, 1 - control);
          apply_speed(right, 1 + control);
        
        elseif strcmp(turn, 'left')
          apply_speed(left, 1 + control);
          apply_speed(right, 1 - control);
        end
          
        
        
        if is_on_line(m, n, pose) == true & (calculateDistance(pose(1), pose(2), goal(1), goal(2)) < hit_point_dist) & (calculateDistance(pose(1), pose(2), start_point(1), start_point(2))>0.1)
          apply_speed(left, 0);
          apply_speed(right, 0);
          
          state = "align";
          fprintf("State changed to: %s\n", state)
          fprintf(log_file,"State changed to: %s\n", state);
        end
    end

  if get_camera==true
    rgb=wb_camera_get_image(camera);

    figure(1); 
    image(rgb);
    axis('off');
    title('Camera image');
  
        % flush graphics
    drawnow;
    
  end
      
end

time_data=transpose(time_data);
distance_to_target=transpose(distance_to_target);

file_name = sprintf('bug2_data_Kf=%.1f_turn_%s.mat', K_follow, turn);

save(file_name, 'time_data', 'distance_to_target', 'position_data', 'lidar_data');

wb_robot_cleanup();
wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);

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
    
function result = calculateDistance(x1, y1, x2, y2)
  result = sqrt((x2-x1)^2 + (y2-y1)^2);
end

function result = align_to_goal(heading, pose, goal, left_motor, right_motor)
  
  Kh = 1.1;
  threshold = 0.01;
  
  thetak = atan2(goal(2)-pose(2), goal(1)-pose(1));
  
  dif_angles = angdiff(heading, thetak);
  
  gamma = Kh * dif_angles;
  
  apply_speed(left_motor, -gamma);
  apply_speed(right_motor, +gamma);
  
  if abs(dif_angles)<threshold
    
    result = true;
    
    apply_speed(left_motor, 0);
    apply_speed(right_motor, 0);
    
  else
    result = false;
  end
  
end  

function result = is_on_line(m, n, pose)
  
  threshold=0.01;
  
  y = m*pose(1)+n;
  
  if abs(y - pose(2))<threshold
    result = true;
  else
    result = false;
  end
end 
 
% cleanup code goes here: write data to files, etc.
