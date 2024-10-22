% Always begin by using addpath
% You can always test your algorithm in simulator
addpath("simulator")

% Add the ARUCO detector
% Check the example in the folder
addpath("arucoDetector")
addpath("arucoDetector/include")
addpath("arucoDetector/dictionary")

% Load parameters
load('arucoDetector/dictionary/arucoDict.mat');
load("calibrationSession.mat")
marker_length = 0.075;

cameraParams = calibrationSession.CameraParameters;

simulation = false;
end_time = 240;

% Initialize the pibot connection

if simulation
    % Load true landmarks from a JSON file
    fid = fopen('groundtruth_1016.json');
    raw = fread(fid, inf);
    str = char(raw');
    fclose(fid);
    json_data = jsondecode(str);
    
    % Extract landmark numbers and positions
    field_names = fieldnames(json_data);
    landmark_nums = cellfun(@(x) str2double(x(2:end)), field_names);
    landmark_positions = cell2mat(struct2cell(json_data)');
    
    % Sort landmarks by their numbers
    [true_landmark_nums, sort_idx] = sort(landmark_nums);
    true_landmarks = landmark_positions(:, sort_idx);

    pb = piBotSim("floor_course.jpg", true_landmarks+[1;1]);
    % Start by placing your robot at the start of the line
    pb.place([1;1], 0);
    pb.showLandmarks(true);
else
    pb = PiBot('192.168.50.1');
end

% Initialise your EKF class
EKF = ekf_slam();

blackPixelThreshold = 90;        % Minimum number of black pixels to consider the line as present
consecutiveThreshold = 2;        % Number of consecutive frames below the threshold to confirm end of line
belowThresholdCount = 0;         % Counter for consecutive frames below the threshold
tic;

fig = figure;
% robotPlot = plot(0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
% hold on;
% landmarkPlot = plot(0, 0, 'b*', 'MarkerSize', 8);

robotPlot = plot(0, 0, 'g-', 'LineWidth', 0.5);
hold on;
landmarkPlots = cell(1, 30);  % Assuming maximum of 30 landmarks
lm_cmap = colormap(hsv(30));

% Load and plot track_data
load('track_data.mat');
trackPlot = plot(x_path + 1, y_path + 1, 'k-', 'LineWidth', 1, 'Color','b');

% Initialize robot path plot
robotPathPlot = scatter(1, 1, 3,"black" ,'filled');

axis equal;
xlim([0.5, 5]);
ylim([0.5, 3.7]);
grid off;
title('Robot Path, Landmark Positions, and Track');
xlabel('X (m)');
ylabel('Y (m)');
% legend([robotPlot, trackPlot, robotPathPlot], ...
%        {'Robot', 'Track', 'Robot Path'}, 'Location', 'best');

% Initialize visualization data struct
vis_data = struct('time', {}, 'robot_pos', {}, 'robot_cov', {}, 'landmark_pos', {}, 'landmark_cov', {}, 'landmark_nums', {});

test_dataset = struct('image', {}, 'dt', {});

landmark_ids_rendered = [];

% INITIAL STATE
u = 0;
q = 0;

start_time = tic; % Start timer for the entire run

while(true)
    dt = toc;
    tic;

    % EKF PREDICT
    EKF.predict(dt, u, q);

    % Get the current camera frame
    try
        img = pb.getImage();
    catch e
        disp("Error in image capture:");
        disp(e.message);
        disp(e.stack(1));
        imshow(img)
        break;  % Exit the loop if image capture consistently fails
    end
    test_dataset(end+1).dt = dt;
    test_dataset(end).image = img;


    % measure landmarks and update EKF
    if simulation
        [landmark_centres, marker_nums] = pb.measureLandmarks();
    else
        [marker_nums, landmark_centres, ~] = detectArucoPoses(img, marker_length, cameraParams, arucoDict);
    end

    if ~isempty(marker_nums)
        % Filter out markers with IDs >= 30 and those more than 2m away
        valid_markers = (marker_nums < 30) & (vecnorm(landmark_centres(1:2)) <= 2);
        marker_nums = marker_nums(valid_markers);
        
        if simulation
            landmark_centres = landmark_centres(:, valid_markers);
        else
            landmark_centres = landmark_centres(valid_markers, 1:2)';
        end

        % Call EKF input_measurements
        EKF.update(landmark_centres, marker_nums);

    end

    % Get estimates from EKF
    [robot_pos, robot_cov] = EKF.output_robot();
    [landmarks_pos, landmarks_cov] = EKF.output_landmarks();
    
    % Use the existing figure
    figure(fig);
    
    % Robot position plot
    [robot_ellipse_x,robot_ellipse_y] = error_ellipse(robot_pos,robot_cov);
    set(robotPlot, 'XData', robot_ellipse_x, 'YData', robot_ellipse_y);
    
    % Update robot path plot
    robot_path_x = cellfun(@(x) x(1), {vis_data.robot_pos});
    robot_path_y = cellfun(@(x) x(2), {vis_data.robot_pos});

    set(robotPathPlot, 'XData', robot_path_x, 'YData', robot_path_y);

    for i = 1:size(landmarks_pos, 2)
        if isempty(landmarkPlots{i}) || ~isvalid(landmarkPlots{i})
            landmarkPlots{i} = plot(NaN, NaN, 'Color', lm_cmap(i,:), 'LineWidth', 0.5);
        end
            
        % Extract landmark covariance
        landmark_cov_i = landmarks_cov(2*i-1:2*i, 2*i-1:2*i);
        
        % Calculate ellipse parameters
        [ellipse_x, ellipse_y] = error_ellipse(landmarks_pos(:,i), landmark_cov_i);
        
        % Update ellipse plot
        set(landmarkPlots{i}, 'XData', ellipse_x, 'YData', ellipse_y);
    end
    
    %draw landmark ids    
    %delete old markers
    for i = 1:length(landmark_ids_rendered)
        delete(landmark_ids_rendered(i));
    end
    %draw new markers
    for i = 1:length(landmarks_pos(1,:))
        landmark_ids_rendered(i) = text(landmarks_pos(1,i), landmarks_pos(2,i), num2str(landmark_nums(i)), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'black');
    end

    drawnow;

    % Get the idx2num array from EKF
    idx2num = EKF.idx2num;

    % Store data for later visualization and evaluation
    current_time = toc(start_time);
    vis_data(end+1).time = current_time;
    vis_data(end).robot_pos = robot_pos(1:3);
    vis_data(end).robot_cov = robot_cov(1:2, 1:2);
    vis_data(end).landmark_pos = landmarks_pos;
    vis_data(end).landmark_cov = landmarks_cov;
    vis_data(end).landmark_nums = idx2num;  % Store all landmark numbers using idx2num

    % Binarize the image with a threshold
    gray_img = rgb2gray(img);
    bin_img = ~imbinarize(gray_img, 0.4);

    % Crop the binary image to the bottom third for line detection
    [height, width] = size(bin_img);
    bottom_third_bin_img = bin_img(round(2*height/3):end, :);

    % Count the number of black pixels in the cropped image
    blackPixelCount = sum(bottom_third_bin_img(:));

    % Update the consecutive frame counter based on blackPixelCount
    if blackPixelCount < blackPixelThreshold
        belowThresholdCount = belowThresholdCount + 1;
    else
        belowThresholdCount = 0; % Reset if the count is above threshold
    end

    % Check if the robot has reached the end of the line
   %  if current_time < 5
   %      u = 0;
   %      q = 0;
   %      % Calculate wheel velocities using inverse kinematics
   %      [wl, wr] = inverse_kinematics(u, q);
   %  elseif current_time < 7
   %      u = 0;
   %      q = 0.4;
   %      % Calculate wheel velocities using inverse kinematics
   %      [wl, wr] = inverse_kinematics(u, q);
   %  elseif current_time < 11
   %      u = 0;
   %      q = -0.4;
   %      % Calculate wheel velocities using inverse kinematics
   %      [wl, wr] = inverse_kinematics(u, q);
   % elseif current_time < 13
   %      u = 0;
   %      q = 0.4;
   %      % Calculate wheel velocities using inverse kinematics
   %      [wl, wr] = inverse_kinematics(u, q);

    if current_time > end_time
        break;
    elseif belowThresholdCount <= consecutiveThreshold
        [u, q, wl, wr] = followLineIteration(height, width, bottom_third_bin_img);
    else
        u = 0;
        q = 1;

        % turn 180 degrees
        turn_time = pi/q;
        
        % Calculate wheel velocities using inverse kinematics
        [wl, wr] = inverse_kinematics(u, q);

        % Helper function to round wheel velocities to the nearest lower multiple of 5
        round_to_lower_5 = @(value) 5 * floor(value / 5);
        wl = round_to_lower_5(wl);
        wr = round_to_lower_5(wr);

        % Calculate the actual velocities after rounding
        [u, q] = forward_kinematics(wl, wr);
        pb.setVelocity(wl, wr);
        pause(turn_time);
        wl = 0;
        wr = 0;
    end

    pb.setVelocity(wl, wr);
    
end

% END LINE FOLLOW
pb.stop();

% Save visualization data to a file
save('visualization_data.mat', 'vis_data');

save('test_dataset.mat', 'test_dataset');

% evaluate landmark estimates
rms_error = evaluate_landmarks(vis_data, simulation, pb);
disp("Landmark position RMS error: ");
disp(rms_error);

% plot the trajectory
plot_trajectory(vis_data);

%output txt
generate_txt(idx2num, landmarks_pos);
