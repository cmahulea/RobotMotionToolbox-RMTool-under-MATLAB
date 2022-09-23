%Get randomized final goal locations in the robotarium arena
% final_goal_points = generate_initial_conditions(N_r, ...
%     'Width', robotar.boundaries(2)-robotar.boundaries(1)-robotar.robot_diameter, ...
%     'Height', robotar.boundaries(4)-robotar.boundaries(3)-robotar.robot_diameter, ...
%     'Spacing', 0.5);

figure(3)
final_goal_points = zeros(N_r,3);
for i=1:N_r
    final_goal_points(1,i) = Robots{i}.pos(1)+robotar.boundaries(1);
    final_goal_points(2,i) = Robots{i}.pos(2)+robotar.boundaries(3);
    final_goal_points(3,i) = 0.0;
    
    goal_plot(i).XData = final_goal_points(1,i);
    goal_plot(i).YData = final_goal_points(2,i);
end

% for j = 1:N
%     goal_labels{j}.Position = final_goal_points(1:2, j)-[0.05;0];
%     
%     d(j).XData = final_goal_points(1,j);
%     d(j).YData = final_goal_points(2,j);
%     
%     a(j).XData = final_goal_points(1,j);
%     a(j).YData = final_goal_points(2,j);
%     a(j).UData = 0.12*cos(final_goal_points(3,j));
%     a(j).VData = 0.12*sin(final_goal_points(3,j));
% end

% Get initial location data for while loop condition.
x=robotar.get_poses();



start_time = tic; %The start time to compute time elapsed.



% Plot the iteration and time in the lower left. Note when run on your
% computer, this time is based on your computers simulation time. For a
% better approximation of experiment time on the Robotarium when running
% this simulation on your computer, multiply iteration by 0.033.
% iteration_caption = sprintf('Iteration %d', 0);
% time_caption = sprintf('Total Time Elapsed %0.2f', toc(start_time));
%
% iteration_label = text(-1.5, -0.8, iteration_caption, 'FontSize', font_size, 'Color', 'r', 'FontWeight', 'bold');
% time_label = text(-1.5, -0.9, time_caption, 'FontSize', font_size, 'Color', 'r', 'FontWeight', 'bold');

robotar.step();

while(~init_checker(x, final_goal_points))
    x = robotar.get_poses();
    
    % Update Plotting Information and Locations
    for q = 1:N_r
        current_plot(q).XData = x(1,q);
        current_plot(q).YData = x(2,q);
        
        robot_labels{q}.Position = x(1:2, q) + [-0.15;0.15];
%         robot_details = sprintf('X-Pos: %0.2f \nY-Pos: %0.2f', x(1,q), x(2,q));
%         robot_details_text{q}.String = robot_details;
%         robot_details_text{q}.Position = x(1:2, q) - [0.2;0.25];
    end
    
    dxu = controller(x, final_goal_points);
    dxu = unicycle_barrier_certificate(dxu, x);
    
    robotar.set_velocities(1:N_r, dxu);
    robotar.step();
    
    
    % Update Iteration and Time marker
%     iteration_caption = sprintf('Iteration %d', i);
%     time_caption = sprintf('Total Time Elapsed %0.2f', toc(start_time));
    
%     iteration_label.String = iteration_caption;
%     time_label.String = time_caption;
    
    % Resize Marker Sizes (In case user changes simulated figure window
    % size, this is unnecessary in submission as the figure window
    % does not change size).
    
%     marker_size_goal = num2cell(ones(1,N_r)*determine_marker_size(robotar, 0.20));
%     [d.MarkerSize] = marker_size_goal{:};
%     marker_size_robot = num2cell(ones(1,N_r)*determine_robot_marker_size(robotar));
%     [g.MarkerSize] = marker_size_robot{:};
%     font_size = determine_font_size(r, 0.05);
%     iteration_label.FontSize = font_size;
%     time_label.FontSize = font_size;
    
%     for k = 1:N_r
        % Have to update font in loop for some conversion reasons.
        % Again this is unnecessary when submitting as the figure
        % window does not change size when deployed on the Robotarium.
%         robot_labels{k}.FontSize = font_size;
%         goal_labels{k}.FontSize = font_size;
%         robot_details_text{k}.FontSize = font_size;
%     end
    
end


