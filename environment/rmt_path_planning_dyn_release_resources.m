%    This is part of RMTool - Robot Motion Toolbox, for Matlab 2010b or newer.
%
%    Copyright (C) 2016 RMTool developing team. For people, details and citing
%    information, please see: http://webdiis.unizar.es/RMTool/index.html.
%
%    This program is free software: you can redistribute it and/or modify
%    it under the terms of the GNU General Public License as published by
%    the Free Software Foundation, either version 3 of the License, or
%    (at your option) any later version.
%
%    This program is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%    GNU General Public License for more details.
%
%    You should have received a copy of the GNU General Public License
%    along with this program.  If not, see <http://www.gnu.org/licenses/>.

%% ============================================================================
%   MOBILE ROBOT TOOLBOX
%   Graphical User Interface
%   First version released on November, 2018.
%   Last modification November 10, 2018.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function [new_Run_cells, new_rob_traj, message2] = rmt_path_planning_dyn_release_resources(Run_cells, rob_traj, data, message)
% %dynamical release of resources
% input data:
%   Run_cells - cells crossed by each robot
%   rob_traj - positions of robots thoughout the paths
%   common - string variable: 'all' - considers the common cells for all
%   trajectories/ 'pairs' - considers the common cells between 2 adjacent
%   trajectories (r_i, r_i+1)
% output data:
%   new_run_cells - new variable which stores the running cells for all
%   trajectories, with few idle states (one idle state is represented by
%   the waiting of one robot until the previous robot crosses the common
%   cells
%   pos_rob - new variable which stores the position of the robots in the
%   trajectories, based on the considered idle states

% find the orer of the robots
idx_end_val = zeros(1,length(rob_traj));
for r = 1:length(rob_traj)
    end_value = Run_cells(r,end);
    idx_end_val(1,r) = find(Run_cells(r,:) == end_value, 1,'first');
end

[idx_end_val, nr_rob] = sort(idx_end_val(1,:));
idx_end_val = [idx_end_val; nr_rob];

message2 = sprintf('%s===========\n The order of the robots is: ', message);
for i = 1:size(idx_end_val,2)
    message2 = sprintf('%s %d ', message2, idx_end_val(2,i));
end

No_r = size(Run_cells,1);

order_rob = cell(1,size(data.Pre,1));
empty_cells = [];
for i = 1:size(data.Pre,1)
    for j = 1:No_r % find the order in which the robots cross each cell
        temp_idx = find(Run_cells(j,:) == i,1,'first');
        if ~isempty(temp_idx)
            order_rob{i} = [order_rob{i}; temp_idx j];
        end
    end
    if ~isempty(order_rob{i})
        idx = sortrows(order_rob{i});
        order_rob{i} = idx; % first column represent the index in the robot trajectory, the second column represent the order of the robots through cell i along the trajectory
        order_rob{i}(:,1) = []; % keep only the order of the robots in the current cell i
    else
        empty_cells = [empty_cells i]; % memorize the empty cells in which no robot cross
    end
end

temp_rob_traj = rob_traj;
% eliminate the duplicate cells in the robot trajectories for an easier
% manipulation
for i = 1:size(Run_cells,1)
    temp_Run_cells = Run_cells(i,:);
    for j = size(Run_cells,2):-1:2
        if (isempty(setxor(Run_cells(i,j),Run_cells(i,j-1))))
            temp_Run_cells(j) = [];
            temp_rob_traj{i}(:,j) = [];
        end
    end
    unique_Run_cells{i} = temp_Run_cells;
end

flag_end_traj = zeros(1,No_r);
idx_rob_traj = 2*ones(1,No_r); % index in the robots trajectory for each robot

% memorize the final cells for each trajectory
final_cell_traj = zeros(1,No_r);
final_rob_traj = zeros(2,No_r);
prev_final_rob_traj = zeros(2,No_r);
for r = 1:No_r
    final_cell_traj(r) = unique_Run_cells{r}(end);
    final_rob_traj(:,r) = temp_rob_traj{r}(:,end);
    prev_final_rob_traj(:,r) = temp_rob_traj{r}(:,end - 1);
end

current_pos_all_rob = [];

% initial positions for the robots
for r = 1:No_r
    new_Run_cells{r}(1) = unique_Run_cells{r}(1);
    unique_Run_cells{r}(1) = [];
    current_pos_all_rob(r) = new_Run_cells{r}(1);
    new_rob_traj{r}(:,1) = temp_rob_traj{r}(:,1);
    temp_rob_traj{r}(:,1) = [];
end

% compute the updated positions of the robot considering their order
% through the cells
while ~isempty(setdiff(flag_end_traj, ones(1,length(unique_Run_cells))))
    for r = 1:length(unique_Run_cells)
        if ~isempty(unique_Run_cells{r}) % the robot advance as long as it still has cells to cross
            current_cell = unique_Run_cells{r}(1); % access the current index in the robot's trajectory
            idx_current_cell = find(order_rob{current_cell} == r); % compute the order of the robot in the current cell
            
            if idx_current_cell == 1 % if is the first one to cross the current cell, then update with the new position
                new_Run_cells{r}(idx_rob_traj(r)) = unique_Run_cells{r}(1); % the robot advance in the next cell
                new_rob_traj{r}(:,idx_rob_traj(r)) = temp_rob_traj{r}(:,1);
                unique_Run_cells{r}(1) = [];
                temp_rob_traj{r}(:,1) = [];
                if isempty(find(current_pos_all_rob(1:end) == current_cell, 1)) && ~isempty(setdiff(new_Run_cells{r}(idx_rob_traj(r)),new_Run_cells{r}(idx_rob_traj(r) - 1)))
                    order_rob{new_Run_cells{r}(idx_rob_traj(r) - 1)}(1) = []; % the previous cell is released only when no other robot occupies the current cell and the first robot of that cell moved into a new cell 
                end
            else % if the robot is not the first one in the current cell, the robot stays in the previous cell
                new_Run_cells{r}(idx_rob_traj(r)) = new_Run_cells{r}(idx_rob_traj(r) - 1); % the robot stays in the same cell
                new_rob_traj{r}(:,idx_rob_traj(r)) = temp_rob_traj{r}(:,1);
            end
            
            % update the screenshot based on the current position of all
            % the team
            current_pos_all_rob(No_r + 1) = new_Run_cells{r}(idx_rob_traj(r));
            current_pos_all_rob(1) = [];

            if new_Run_cells{r}(idx_rob_traj(r)) == final_cell_traj(r) && flag_end_traj(r) == 0
                flag_end_traj(r) = 1; % checked if the robot arrived to the destination cell
            end
        end
        if isempty(unique_Run_cells{r})  % if the robot does not move from is initial position, that means the robot is already in his final cell
            flag_end_traj(r) = 1;
        end
        if flag_end_traj(r) == 1 
            new_Run_cells{r}(idx_rob_traj(r)) = final_cell_traj(r);
            new_rob_traj{r}(:,idx_rob_traj(r)) = final_rob_traj(:,r);
            current_pos_all_rob(No_r + 1) = new_Run_cells{r}(idx_rob_traj(r));
            current_pos_all_rob(1) = [];
        end

        idx_rob_traj(r) = idx_rob_traj(r) + 1; % increase index in robot's trajectory

    end
end

% add the anterior position of the final position for the correct plot of the robot trajectories
for i = 1:No_r
    k = 0;
    for j = size(new_rob_traj{i},2):-1:1
        if new_rob_traj{i}(:,j) == final_rob_traj(:,i)
            new_rob_traj{i}(:,j) = [];
            k = k + 1;
        end
    end
    temp_length = length(new_rob_traj{i});   
    new_rob_traj{i}(:,end+1) = prev_final_rob_traj(:,i);
    aux = new_rob_traj{i};
    temp_length_run_cells = length(new_Run_cells{i});
    second_aux = repmat(final_rob_traj(:,i),1,temp_length_run_cells - temp_length);
    new_rob_traj{i} = [];
    new_rob_traj{i} = [aux second_aux];
end

% initial figures (empty environment)
% name_fig = 'InitTrajBoolSpec.fig';
% init_fig = openfig(name_fig)';
% data.rob_plot.line_color = {'r','b','m','g','c','k','y',[0.8500 0.3250 0.0980],[0.4940 0.1840 0.5560],[0.6350 0.0780 0.1840],[0 0.4470 0.7410]};

% parallel movement of the robots
for uu = 1:length(new_rob_traj{1})-1
    for rr = 1:length(new_rob_traj)
        plot(new_rob_traj{rr}(1,uu:uu+1),new_rob_traj{rr}(2,uu:uu+1),'Color',data.rob_plot.line_color{rr},'LineWidth',data.rob_plot.line_width{rr});
        h = plot(new_rob_traj{rr}(1,uu+1),new_rob_traj{rr}(2,uu+1),'Color',data.rob_plot.line_color{rr},...
            'Marker',data.rob_plot.marker{rr},'LineWidth',data.rob_plot.line_width{rr});
        set(h,'XData',new_rob_traj{rr}(1,uu+1),'YData',new_rob_traj{rr}(2,uu+1));
        if uu == 1 % mark the start point
            plot(new_rob_traj{rr}(1,uu),new_rob_traj{rr}(2,uu),'Color',data.rob_plot.line_color{rr},...
                'Marker',data.rob_plot.marker{rr},'LineWidth',data.rob_plot.line_width{rr});
        elseif uu == size(new_rob_traj{1},2)-1 % mark the end point
            plot(new_rob_traj{rr}(1,end),new_rob_traj{rr}(2,end),'Color',data.rob_plot.line_color{rr},...
                'Marker',data.rob_plot.marker{rr},'LineWidth',data.rob_plot.line_width{rr},'Color','r');
        end
    end

    pause(1);
end

% save the robot trajectories to save it in the txt file
message2 = sprintf('%s\n\nSOLUTION - runs of robots: \n',message2);
for j = 1 : length(new_Run_cells)
    message2 = sprintf('%s\nRobot %d: ',message2,j);
    temp = new_Run_cells{j};
    for k = 1 : length(temp)-1
        message2 = sprintf('%s%d,',message2,temp(k));
    end
    message2 = sprintf('%s%d',message2,temp(length(temp)));
end
