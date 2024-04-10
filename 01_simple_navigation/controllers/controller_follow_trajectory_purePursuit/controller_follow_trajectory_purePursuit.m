% MATLAB controller for Webots
% File: controller_vete_punto.m
% Date:
% Description:
% Author:
% Modifications: juan

function my_controller

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

TIME_STEP = 64;
CAMERA_TIME_STEP=256;

MOTOR_MAX_SPEED=6.28;

%set this parameter as true to enable and view camera (reduces performance)
get_camera = false;

%set this parameter as true to view live graphs (reduces performance a lot)
enable_live_graphs = false; 


%Open (or create if it doesn't exist, a log file)
log_file = fopen('log.txt', 'w');
if log_file == -1
    error('Could not open the log file.');
end

% Get the supervisor node
supervisor = wb_supervisor_node_get_from_def('E-PUCK');
if supervisor == -1
    error('Could not find the Supervisor node.');
end

% Get the arena node
arena_node = wb_supervisor_node_get_from_def('ARENA');
if arena_node == -1
    error('Could not find the arena node.');
end


% Get the arena size field from the PROTO node
arena_size_field = wb_supervisor_node_get_field(arena_node, 'floorSize');
if arena_size_field == -1
    error('Could not find the arena size field.');
end


% Get the floor size vector
arena_size = wb_supervisor_field_get_sf_vec2f(arena_size_field);
arena_size_x = arena_size(1);
arena_size_y = arena_size(2);


% Display the tile size
fprintf('Arena size: %.2f x %.2f m \n', arena_size_x, arena_size_y);


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

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%

%trajectory data for equation defined in 
x_traj = linspace(0, 3, 100);
y_traj = 1/3*sin(3*x_traj);


%trajectory points matrix

trajectory = [x_traj', y_traj'];

goal=trajectory(end, :);

Kv = 0.5;
Kh = 1;


%look ahead distance
L=1;

target_point=[0, 0];

step=0;
pursuit_step=2;

proximity_threshold = 0.1;

time_data=[];
position_data=[];

speed=[];

fprintf('-----------------------------------------------------------------------------------------------------\n');
fprintf('  Time (s) |  X (m) |  Y (m) | Goal angle (deg) | Actual angle (deg) | V_Left (m/s) | V_Right (m/s) \n');
fprintf('-----------------------------------------------------------------------------------------------------\n');

fprintf(log_file, '-----------------------------------------------------------------------------------------------------\n');
fprintf(log_file, '  Time (s) |  X (m) |  Y (m) | Goal angle (deg) | Actual angle (deg) | V_Left (m/s) | V_Right (m/s) \n');
fprintf(log_file, '-----------------------------------------------------------------------------------------------------\n');


while wb_robot_step(TIME_STEP) ~= -1
  
  step=step+1;
  
  time_data=[time_data, wb_robot_get_time()];
  
  pose = wb_gps_get_values(gps);
  xk = pose(1);
  yk = pose(2);
  
  position_data=[position_data; xk, yk];
  
  [target_point, pursuit_step] = purePursuit2(pursuit_step, position_data(step, :), trajectory, L);
  
  angle = wb_compass_get_values(compass);
  theta_ = pi/2-atan2(angle(2), angle(1));
  
  thetak = atan2(target_point(2) - pose(2), target_point(1) - pose(1));
  
  error_speed=sqrt((target_point(1)-pose(1))^2+(target_point(2)-pose(2))^2);
  
  %control variables: v_ y gamma_
  v_ = Kv*error_speed;
  
  dif_angles = angdiff(theta_, thetak);

  gamma_ = Kh*dif_angles;
       
  speed(step, 1) = v_;
  speed(step, 2) = v_;
  
  speed(step, 1) = speed(step, 1) - gamma_;
  speed(step, 2) = speed(step, 2) + gamma_;
  
  if ((abs(speed(step,1)) > MOTOR_MAX_SPEED) || (abs(speed(step,2)) > MOTOR_MAX_SPEED))
    ratio = MOTOR_MAX_SPEED/max([abs(speed(step,1)) abs(speed(step,2))]);
    speed(step,1) = speed(step,1) * ratio;
    speed(step,2) = speed(step,2) * ratio;
  end 
  
  wb_motor_set_velocity(left, speed(step,1));
  wb_motor_set_velocity(right, speed(step, 2));
    
  if get_camera==true
    rgb=wb_camera_get_image(camera);
  end
  
    
  fprintf('%10.3f | %6.3f | %6.3f | %16.3f | %18.3f | %12.3f | %13.3f\n', ...
        wb_robot_get_time(), position_data(step,1), position_data(step,2), ...
        theta_ * 180/pi, thetak * 180/pi, speed(step,1), speed(step,2));
  
  %Write the data into the log file  
  fprintf(log_file, '%10.3f | %6.3f | %6.3f | %16.3f | %18.3f | %12.3f | %13.3f\n', ...
        wb_robot_get_time(), position_data(step,1), position_data(step,2), ...
        theta_ * 180/pi, thetak * 180/pi, speed(step,1), speed(step,2));  
     
    
  % Verify if the robot is sufficiently close to the target
  if calculateDistance(position_data(step,:), goal) < proximity_threshold
    wb_motor_set_velocity(left, 0.0);  % Stop left motor
    wb_motor_set_velocity(right, 0.0); % Stop right motor
    display('Target reached!')
    fprintf(log_file, 'Target reached!');
    
    break; % Exit loop
  end
  
  if enable_live_graphs == true  
    %plot distance to target       
    subplot(2,2,1);
    plot(time_data, distance_to_target, '-r');
    xlabel('Time (s)'); 
    ylabel('Distance (m)');
    title('Distance to target');
    
    subplot(2,2,2);
    
    hold on;
    
    plot([position_data(1,1), xg], [position_data(1,2), yg], '-b');
    
    plot(position_data(:,1), position_data(:,2), '-r');
    
    hold off;
    axis auto;
    xlabel('X axis (m)'); 
    ylabel('Y axis (m)');
    title('Trajectory');
    
    if get_camera==true
      subplot(2, 2, [3 4])   
      image(rgb);
      axis('off');
      title('Camera image');
    end
    
    % flush graphics
    drawnow;
  
  end

%%%%%  
end

%Close the log file
fclose(log_file);

time_data=transpose(time_data);

file_name = sprintf('matrixes_Kv=%.1f_Kh=%.1f_L=%.2f.mat', Kv, Kh, L);

save(file_name, 'time_data', 'position_data');

if enable_live_graphs == true 
  saveas(gcf, 'graphics.png');
  close all;
end

wb_robot_cleanup();
wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);

end

function dist = calculateDistance(pi, pf)
  dx = pf(1)-pi(1);
  dy = pf(2)-pi(2);

  dist = sqrt(dx^2 + dy^2);
end

function [target_point, pursuit_step] = purePursuit2(pursuit_step, pose, trajectory, L)
  
    threshold = 0.01; 
    
    if calculateDistance(pose, trajectory(pursuit_step, :)) < threshold
      pursuit_step = pursuit_step + 1;
    end
    
    direction_vector=[trajectory(pursuit_step, 1)-pose(1), trajectory(pursuit_step, 2)-pose(2)];
    
    magnitude = sqrt(direction_vector(1)^2 + direction_vector(2)^2);
    
    normalized_direction = [direction_vector(1)/magnitude, direction_vector(2)/magnitude];
    
    target_point = [pose(1)+L*normalized_direction(1), pose(2)+L*normalized_direction(2)];
end
    
% cleanup code goes here: write data to files, etc.
