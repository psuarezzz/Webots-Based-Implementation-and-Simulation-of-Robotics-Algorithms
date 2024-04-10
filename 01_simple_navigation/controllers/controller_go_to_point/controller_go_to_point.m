% MATLAB controller for Webots
% File: controller_vete_punto.m
% Date:
% Description:
% Author:
% Modifications:
function my_controller

TIME_STEP = 64;
CAMERA_TIME_STEP=256;

MOTOR_MAX_SPEED=6.28;

%set this parameter as true to enable and view camera (reduces performance)
get_camera = false;

%set this parameter as true to view live graphs (reduces performance a lot)
enable_live_graphs = false; 

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
fprintf('Arena size: %.2f x %.2f\n', arena_size_x, arena_size_y);


%get and enable devices, e.g.:
if get_camera==true
  camera = wb_robot_get_device('camera');
  wb_camera_enable(camera, CAMERA_TIME_STEP);
end;

log_file = fopen('log.txt', 'w');
if log_file == -1
    error('Could not open the log file.');
end


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

%Set goal point coordinates
xg = 2;
yg = 2;

% Display the entered goal coordinates
fprintf('Destination point: (%.3f, %.3f)\n', xg, yg);

Kv = 5;
Kh = 2;

step=0;

%Distance to objective set it as reached
proximity_threshold = 0.01;

distance_to_target = [];
time_data=[];
position_data=[];

disp('----------------------------------------------------------------------------------------------------');
disp('  Time (s) |  X (m) |  Y (m) | Goal angle (deg) | Actual angle (deg) | V_Left (m/s) | V_Right (m/s) |');
disp('----------------------------------------------------------------------------------------------------');

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

  angle = wb_compass_get_values(compass);
  theta_ = pi/2-atan2(angle(2), angle(1));
    
  %distance_to_target(step)= sqrt((xg-xk)^2+(yg-yk)^2);
  distance_to_target(step)= sqrt((xg-position_data(step,1))^2+(yg-position_data(step,2))^2);
  
  thetak = atan2(yg-position_data(step,2), xg-position_data(step,1));
 
  %control variables: v_ y gamma_
  v_ = Kv*distance_to_target(step);
  
  dif_angulos = angdiff(theta_, thetak);
  gamma_ = Kh*dif_angulos;
      
      
  v_right = v_;
  v_left = v_;
  
  v_right = v_right + gamma_;
  v_left = v_left-gamma_;
  
  if ((abs(v_left) > MOTOR_MAX_SPEED) || (abs(v_right) > MOTOR_MAX_SPEED))
  	ratio = MOTOR_MAX_SPEED/max([abs(v_left) abs(v_right)]);
  	v_left = v_left * ratio;
  	v_right = v_right * ratio;
  end
    
  wb_motor_set_velocity(left, v_left);
  wb_motor_set_velocity(right, v_right);
  
  if get_camera==true
    rgb=wb_camera_get_image(camera);
  end
  
 
      disp([sprintf('%10.3f |', wb_robot_get_time()), ...
          sprintf(' %6.3f |', position_data(step,1)), ...
          sprintf(' %6.3f |', position_data(step,2)), ...
          sprintf(' %16.3f |', theta_ * 180/pi), ...
          sprintf(' %18.3f |', thetak * 180/pi), ...
          sprintf(' %12.3f |', v_left), ...
          sprintf(' %13.3f |', v_right)]);
          
       %Write the data into the log file  
  fprintf(log_file, '%10.3f | %6.3f | %6.3f | %16.3f | %18.3f | %12.3f | %13.3f\n', ...
        wb_robot_get_time(), position_data(step,1), position_data(step,2), ...
        theta_ * 180/pi, thetak * 180/pi, v_left, v_right);  
 
  % Verify if the robot is sufficiently close to the target
  if distance_to_target(step) < proximity_threshold
    wb_motor_set_velocity(left, 0.0);  % Stop left motor
    wb_motor_set_velocity(right, 0.0); % Stop right motor
    disp('Target reached!')
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
 
end

time_data=transpose(time_data);
distance_to_target=transpose(distance_to_target);

file_name = sprintf('matrixes_Kv=%.1f_Kh=%.1f.mat', Kv, Kh);

save(file_name, 'time_data', 'distance_to_target', 'position_data');

%Close the log file
fclose(log_file);

if enable_live_graphs == true 
  saveas(gcf, 'graphics.png');
  close all;
end

wb_robot_cleanup();
wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);

end

