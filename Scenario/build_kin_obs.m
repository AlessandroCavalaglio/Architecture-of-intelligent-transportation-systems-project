function build_kin_obs()

    % ----------------------------------
    %% Define the layout of the double lane change scenario
    % ----------------------------------
    road_width = 6;

    % Starting point 
    x_path = 0;
    y_path = 0;
    theta_path = 0;
    
    % ------------------
    % Initial straight line
    % ------------------
    length_initial_straight = 100;
    x_path = [x_path; length_initial_straight];
    y_path = [y_path; 0];
    theta_path = [theta_path; 0];
    
    % ------------------
    %% Plot the scenario 
    % ------------------
    % Build the clothoid list 
    S_clothoid = ClothoidList; 
    for ii=1:length(x_path)-1
        S_clothoid.push_back_G1(x_path(ii),y_path(ii),theta_path(ii), x_path(ii+1),y_path(ii+1),theta_path(ii+1));
    end
    % Evaluate the points belonging to the path (clothoid list)
    curv_absc_sample = 0:0.1:S_clothoid.length;
    [x_path_sample,y_path_sample,~,~] = S_clothoid.evaluate(curv_absc_sample);
    [x_left_path_sample,y_left_path_sample,~,~] = S_clothoid.evaluate(curv_absc_sample, road_width/2);
    [x_right_path_sample,y_right_path_sample,~,~] = S_clothoid.evaluate(curv_absc_sample, -road_width/2);
    
    figure('Name','Scenario','NumberTitle','off'), clf  
    hold on
    axis equal
    S_clothoid.plot_offs(road_width/2,numel(curv_absc_sample));
    S_clothoid.plot;
    S_clothoid.plot_offs(-road_width/2,numel(curv_absc_sample));
    grid on
    xlabel('x [m]')
    ylabel('y [m]')
    title('Circuit')
    
    % ------------------
    %% Save the points of the path
    % ------------------
    path.x = x_path;
    path.y = y_path;
    path.theta = theta_path;
    path.x_sampled = x_path_sample;
    path.y_sampled = y_path_sample;
    path.x_left_sampled = x_left_path_sample;
    path.y_left_sampled = y_left_path_sample;
    path.x_right_sampled = x_right_path_sample;
    path.y_right_sampled = y_right_path_sample;
    % Ground 
    path.road_condition = [1,1,1,1]; %rr,rl,fr,fl
    
    % Simulation length
    times.t0        = 0;     % [s]  <--- starting time
    times.step_size = 1e-4; % [s]  <--- discrete solver step
    times.tf        = 20;    % [s]  <--- stop simulation time
    
    % Initial speed
    vehicle_control.max_speed = 70;
    
    % Direct control
    vehicle_control.low_level_control = 1;
    vehicle_control.time_pedal   = [0, 0.8, 0.8 + 0.1, times.tf];
    vehicle_control.req_pedal = [0.3, 0.3, 0.3, 0.3];
    
    rise_time  = vehicle_control.max_speed/50; 
    % Vector of times where to switch value of the input signal
    vehicle_control.time_manoeuvre   = [0, 1, 1 + rise_time, times.tf];

    % Output value of the input signal at the corresponding time
    vehicle_control.req_speed = [vehicle_control.max_speed, vehicle_control.max_speed,  vehicle_control.max_speed, vehicle_control.max_speed];
    
    % Lat control
    vehicle_control.low_level_control_steer = 0;
    vehicle_control.time_delta   = [0, times.tf];
    vehicle_control.req_delta = [60, 60];
    save('./Scenario/kin_obs_road_1','path','times','vehicle_control')
    
    
end