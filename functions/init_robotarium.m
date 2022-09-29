current_folder = cd;
cd robotarium-matlab-simulator-master
init
cd(current_folder);

initial_positions = generate_initial_conditions(N_r, 'Spacing', 0.5);
robotar = Robotarium('NumberOfRobots', N_r, 'ShowFigure', true, 'InitialConditions', initial_positions);

final_goal_points = generate_initial_conditions(N_r, ...
    'Width', robotar.boundaries(2)-robotar.boundaries(1)-robotar.robot_diameter, ...
    'Height', robotar.boundaries(4)-robotar.boundaries(3)-robotar.robot_diameter, ...
    'Spacing', 0.5);

% Plotting Initialization
% Color Vector for Plotting
% Note the Robotarium MATLAB instance runs in a docker container which will
% produce the same rng value every time unless seeded by the userobotar.
CM = rand(N_r,3);%{'.-k','.-b','.-r','.-g','.-m','.-y','.-c'};


%Marker, font, and line sizes
marker_size_goal = determine_marker_size(robotar, 0.20);
marker_size_robot = determine_robot_marker_size(robotar);
font_size = determine_font_size(robotar, 0.05);
line_width = 5;


% Create a barrier certificate for use with the above parameters
unicycle_barrier_certificate = create_uni_barrier_certificate_with_boundary();
args = {'PositionError', 0.025, 'RotationError', 50};
init_checker = create_is_initialized(args{:});
controller = create_waypoint_controller(args{:});

% x=robotar.get_poses();
for i = 1:N_r
    % Initialize additional information plot here. Note the order of
    % plotting matters, objects that are plotted later will appear over
    % object plotted previously.
    
    % Text for robot identification
    robot_caption = sprintf('Robot %d', i);
    % Text with robot position information
    %robot_details = sprintf('X-Pos: %d \nY-Pos: %d', x(1,i), x(2,i));
    % Text with goal identification
    goal_caption = sprintf('G%d', i);
    % Plot colored square for goal location.
    goal_plot(i) = plot(final_goal_points(1,i), final_goal_points(2,i),'s','MarkerSize',marker_size_goal,'LineWidth',line_width,'Color',CM(i,:));
    % Plot the arrow indicating goal orientation.
%     a(i) = quiver(final_goal_points(1,i), final_goal_points(2,i), 0.12*cos(final_goal_points(3,i)), 0.12*sin(final_goal_points(3,i)), 'LineWidth', line_width, 'MaxHeadSize', 2*line_width, 'Color',CM(i,:));
    % Plot the goal identification text inside the goal location
    %     goal_labels{i} = text(final_goal_points(1,i)-0.05, final_goal_points(2,i), goal_caption, 'FontSize', font_size, 'FontWeight', 'bold');
    % Plot colored circles showing robot location.
    current_plot(i) = plot(initial_positions(1,i),initial_positions(2,i),'o','MarkerSize', marker_size_robot,'LineWidth',5,'Color',CM(i,:));
    % Plot the robot label text
    robot_labels{i} = text(500, 500, robot_caption, 'FontSize', font_size, 'FontWeight', 'bold');
    % Plot the robot position information text
    %robot_details_text{i} = text(500, 500, robot_details, 'FontSize', font_size, 'FontWeight', 'bold');
end

% Import and scale the GT logo appropriately.
gt_img = imread('mapa1.png'); % Original input image file

% Display the image with an associated spatial referencing object.
x_img = linspace(-1.6, 1.6, size(gt_img,2));
y_img = linspace(1.0, -1.0, size(gt_img,1)); %Note the 1 to -1 here due to the (1,1) pixel being in the top left cornerobotar.
gt_img_handle = image(x_img, y_img, gt_img,'CDataMapping','scaled');

% We can change the order of plotting priority, we will plot goals on the
% bottom and have the iteration/time caption on top.
% uistack(a,'bottom'); %Arrows are at the very bottom.
% uistack([goal_labels{:}], 'bottom'); % Goal labels are at the very bottom, arrows are now only above the goal labels.
uistack(current_plot, 'bottom');% Goal squares are at the very bottom, goal labels are above the squares and goal arrows are above those.
% uistack([iteration_label], 'top'); % Iteration label is on top.
% uistack([time_label], 'top'); % Time label is above iteration label.
uistack(gt_img_handle, 'bottom'); % Image under everything else.
