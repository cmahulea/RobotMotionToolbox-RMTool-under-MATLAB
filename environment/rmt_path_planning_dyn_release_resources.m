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

function [new_Run_cells, new_rob_traj, message] = rmt_path_planning_dyn_release_resources(Run_cells, data, message,obstacles)
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

No_r = size(Run_cells,1);
flag_end_traj = zeros(1,No_r);
idx_rob_traj = ones(1,No_r); % index in the robots trajectory for each robot
current_pos_all_rob = Run_cells(:,1); % initial cell of all robots

Pre = data.Pre;
Post = data.Post;

cell_obs = cell2mat(data.T.props(obstacles)); % cells which are considered obstacles and should be avoided
inhib_trans = [];
for i = 1:length(cell_obs)
   inhib_transPre = find(Pre(cell_obs(i),:));
   inhib_transPost = find(Post(cell_obs(i),:));
   inhib_trans = [inhib_trans inhib_transPost inhib_transPre];
end

% inhibit transitions towards/from cells which are considered obstacles
inhib_trans = unique(inhib_trans);
Pre(:,inhib_trans) = []; 
Post(:,inhib_trans) = [];

data.relres.Pre = Pre; % save update Pre and Post for re-plan trajectories function (relres = release resources)
data.relres.Post = Post;

nplaces = size(Pre,1); %number of places
ntrans = size(Pre,2); % number of transitions

if data.optim.param_boolean.UserCount == 0
    message = sprintf('\n%s The selected approach is respected, without a re-planning of trajectories',message);
else
    message = sprintf('\n%s Robot trajectories are re-computed for a number of %d robots waiting to cross a common cell among their trajectories.',message, data.optim.param_boolean.UserCount);
end

% eliminate the duplicate cells in the robot trajectories for an easier
% manipulation
aux_Run_cells = [];
max_length_cell = [];
for i = 1:size(Run_cells,1)
    temp_Run_cells = Run_cells(i,:);
    for j = size(Run_cells,2):-1:2
        if (isempty(setxor(Run_cells(i,j),Run_cells(i,j-1))))
            temp_Run_cells(j) = [];
        end
    end
    unique_Run_cells{i} = temp_Run_cells;
    max_length_cell = [max_length_cell length(unique_Run_cells{i})];
    
end
% make the length of trajectories of the same length by maintaining
% the robot in the final cell
max_length_cell = max(max_length_cell);
for r = 1:No_r
    unique_Run_cells{r}(end:end+max_length_cell - length(unique_Run_cells{r})) = unique_Run_cells{r}(end);
    aux_Run_cells = [aux_Run_cells; unique_Run_cells{r}];
end

% find the order in which the robots cross each cell based on its unique
% trajectory
[order_rob_cell,final_cell_traj,new_Run_cells] = rmt_find_order_trajectories(data,aux_Run_cells,No_r);

% initial positions for the robots
for r = 1:No_r
    unique_Run_cells{r}(1) = [];
end
message = sprintf('\n%s The selected approach is '' same trajectories but different order of robots crossing the common cells'', as long as the selected number of robots waiting to enter a common cell is 0. \n',message);
% end
mf = zeros(size(Pre,1),1);
mf(Run_cells(:,end)) = 1; % final marking in PN
flag_count = 0;
count = 0;
length_RunTraj = zeros(1,No_r);

ss_replan = [];
time_to_replan = [];

% compute the updated positions of the robot considering their order
% through the cells
count_replan = 1;
count_steps = 0;
tic
while ~isempty(setdiff(flag_end_traj, ones(1,length(unique_Run_cells))))
    count_steps = count_steps + 1;% count total number of steps for all robots to reach their destination
    count = 0; % initialize at each step the number of robots which waits to enter a common cell
    for r = 1:length(unique_Run_cells)
        idx_rob_traj(r) = idx_rob_traj(r) + 1; % increase index in robot's trajectory
        cell_idx =[];
        %  check the cells in which the robots are on the same index
        for k = 1:No_r
            if length(new_Run_cells{k}) >= idx_rob_traj(r)
                cell_idx = [cell_idx new_Run_cells{k}(idx_rob_traj(r))];
            end
        end
        
        if ~isempty(unique_Run_cells{r}) % the robot advance as long as it still has cells to cross
            current_cell = unique_Run_cells{r}(1); % access the current index in the robot's trajectory
            idx_current_cell = find(order_rob_cell{current_cell} == r); % compute the order of the robot in the current cell
            previous_cell = new_Run_cells{r}(idx_rob_traj(r) - 1);
            
            if idx_current_cell == 1 && isempty(find(current_pos_all_rob(1:end) == current_cell, 1)) &&  isempty(find(cell_idx == current_cell)) % if is the first one to cross the current cell, then update with the new position
                new_Run_cells{r}(idx_rob_traj(r)) = current_cell; % the robot advance in the next cell
                unique_Run_cells{r}(1) = [];
                if  ~isempty(setdiff(current_cell,previous_cell)) && ~isempty(order_rob_cell{previous_cell})
                    order_rob_cell{previous_cell}(1) = []; % the previous cell is released only when no other robot occupies the current cell and the first robot of that cell moved into a new cell
                end
                
            else % if the robot is not the first one in the current cell, the robot stays in the previous cell
                new_Run_cells{r}(idx_rob_traj(r)) = previous_cell; % the robot stays in the same cell
                count = count + 1; % number of robots which waits to enter a common cell
            end
            
            if new_Run_cells{r}(idx_rob_traj(r)) == final_cell_traj(r) && flag_end_traj(r) == 0
                flag_end_traj(r) = 1; % checked if the robot arrived to the destination cell
            end
        end
        
        current_pos_all_rob(r) = new_Run_cells{r}(end); % update screen shot for each step (current position of robot r)
        
        if isempty(unique_Run_cells{r})  % if the robot does not move from is initial position, that means the robot is already in his final cell
            flag_end_traj(r) = 1;
        end
        
        if flag_end_traj(r) == 1 % if the robot arrives at the destination cell, it stays there until all robot reaches their destination
            new_Run_cells{r}(idx_rob_traj(r)) = final_cell_traj(r);
        end
        length_RunTraj(r) = length(new_Run_cells{r});
        
        % reset flag to re-compute trajectories
        if flag_count == 1 && data.optim.param_boolean.UserCount ~= 0
            flag_count = 0;
        end
        
    end
    %% NEW TRAJECTORIES
    if count == data.optim.param_boolean.UserCount &&  data.optim.param_boolean.UserCount ~= 0 % if the number of robots which waits to enter a common cell is equal with UserCount, then the trajectories are re-planned
        % update the screenshot based on the current position of all
        % the team
        
        current_m0 = zeros(size(Pre,1),1); % update the initial marking based on the current position of the robots
        current_m0(current_pos_all_rob) = 1;
        
        % take a screen-shot every time the trajectories are re-planned
        ss_replan = [ss_replan count_steps]; % necessary for plot
        
        % compute new trajectories for the robots
        tic
        [xmin, message] = rmt_path_planning_pn_new_traj(data,current_m0,mf,message);
        [unique_Run_cells, Runs, message] = rmt_path_planning_boolspec_dif_trajectories(data,xmin,No_r,message);
        tt = toc;
        
        time_to_replan(count_replan) = tt; % memorize the time to compute the new trajectories for all iterations
        
        % find new order of the robots with their new final
        % destination
        [order_rob_cell,~,~] = rmt_find_order_trajectories(data,Runs,No_r);
        
        % update order for new_Run_cells & new_rob_traj
        another_Run_cells = cell(1,No_r);
        another_flag_end = zeros(1,No_r);
        another_idx_rob_traj = zeros(1,No_r);
        current_pos_all_rob = [];
        for k = 1:No_r
            idx = find(new_Run_cells{k}(end) == Runs(:,1));
            another_Run_cells{idx} = new_Run_cells{k};
            another_idx_rob_traj(idx) = length(new_Run_cells{k});
            another_flag_end(idx) = flag_end_traj(k);
            final_cell_traj(k) = unique_Run_cells{k}(end);
            unique_Run_cells{k}(1) = [];
            current_pos_all_rob(k) = new_Run_cells{k}(end);
        end
        
        new_Run_cells = another_Run_cells;
        idx_rob_traj = another_idx_rob_traj;
        flag_end_traj = another_flag_end;
        flag_count = 1; % flag used when the trajectories are re-computed
        count_replan = count_replan + 1;
    end
end
time = toc;

message = sprintf('\n%s The trajectories were re-planned by a number of %d times\n',message, count_replan-1);
% message = sprintf('\n%s Time to follow the trajectories: %d \n',message, time);
message = sprintf('\n%s Re-plan trajectories: mean time: %d and standard deviation time: %d \n',message, mean(time_to_replan),std(time_to_replan));

% make all trajectories of the same length - necessary to plot in parallel
Traj_runs = [];

for r = 1:No_r
    Traj_runs = [Traj_runs; new_Run_cells{r}];
end

new_rob_traj = rmt_rob_cont_traj_new(data.T,Traj_runs,data.initial);

% find the order of the robots based on their original position, useful for
% when the trajectories are re-planned
idx_end_val = zeros(1,No_r);
for r = 1:No_r
    end_value = new_Run_cells{r}(end);
    idx_end_val(1,r) = find(new_Run_cells{r} == end_value, 1,'first');
end

[idx_end_val, nr_rob] = sort(idx_end_val(1,:));
idx_end_val = [idx_end_val; nr_rob];
new_init_cells = Traj_runs(idx_end_val(2,:),1);
new_order_rob = [];
for r = 1:No_r
   new_order_rob = [new_order_rob find(new_init_cells(r) == Run_cells(:,1))];
end

message = sprintf('%s======\n Number of steps for all robots is: %d \n', message, count_steps);
message = sprintf('%s===========\n The order of the robots is: ', message);
for i = 1:length(new_order_rob)
    message = sprintf('%s %d ', message, new_order_rob(i));
end

%% plot trajectories
% initial figures (empty environment)
% 
% name_fig = 'InitTrajBoolSpec.fig';
% init_fig = openfig(name_fig)';

alpha_transparency = 0.5;
color_transparency = {[1,0,0,alpha_transparency], [0,0,1,alpha_transparency], [1,0,1,alpha_transparency],...
    [0,1,0,alpha_transparency], [0,1,1,alpha_transparency], [0,0,0,alpha_transparency], [1,1,0,alpha_transparency],...
    [0.8500 0.3250 0.0980,alpha_transparency],[0.4940 0.1840 0.5560,alpha_transparency],...
    [0.6350 0.0780 0.1840,alpha_transparency],[0 0.4470 0.7410,alpha_transparency]};
hh_v = [];
hh_m = [];
no_cell_plot = 0;
% parallel movement of the robots
for uu = 1:length(new_rob_traj{1})-1
    for rr = 1:length(new_rob_traj)
        % color last 2 cells of each trajectory and add the current
        % position of the robot
        current_cell_traj = Traj_runs(rr,uu);
        hh = fill(data.T.Vert{current_cell_traj}(1,:),data.T.Vert{current_cell_traj}(2,:),data.rob_plot.line_color{rr},'FaceAlpha',0.2,'EdgeColor',data.rob_plot.line_color{rr});
        set(hh,'XData', data.T.Vert{current_cell_traj}(1,:), 'YData',data.T.Vert{current_cell_traj}(2,:), 'FaceAlpha', 0.2);
        
        hh_marker = plot(mean(data.T.Vert{current_cell_traj}(1,:)),mean( data.T.Vert{current_cell_traj}(2,:)),'Color',data.rob_plot.line_color{rr},...
            'Marker',data.rob_plot.marker{rr},'LineWidth',data.rob_plot.line_width{rr});
        set(hh_marker,'XData', mean(data.T.Vert{current_cell_traj}(1,:)), 'YData',mean( data.T.Vert{current_cell_traj}(2,:)),'Marker',data.rob_plot.marker{rr});
        hh_v{rr} = hh;
        hh_m{rr} = hh_marker;
        if uu == 1 % mark the start point
            plot(new_rob_traj{rr}(1,uu),new_rob_traj{rr}(2,uu),'Color',data.rob_plot.line_color{rr},...
                'Marker',data.rob_plot.marker{rr},'LineWidth',data.rob_plot.line_width{rr});
            
        elseif uu == size(new_rob_traj{rr},2)-1 % mark the end point
            plot(new_rob_traj{rr}(1,end),new_rob_traj{rr}(2,end),'Color',data.rob_plot.line_color{rr},...
                'Marker',data.rob_plot.marker{rr},'LineWidth',data.rob_plot.line_width{rr},'Color','r');
        elseif ~isempty(find(ss_replan == uu,1)) % plot the moments where the replanning is made
            fill(data.T.Vert{current_cell_traj}(1,:),data.T.Vert{current_cell_traj}(2,:),data.rob_plot.line_color{rr},'FaceAlpha',0.1,'EdgeColor',data.rob_plot.line_color{rr});
            plot(mean(data.T.Vert{current_cell_traj}(1,:)),mean( data.T.Vert{current_cell_traj}(2,:)),'Color',data.rob_plot.line_color{rr},...
                'Marker','d','LineWidth',data.rob_plot.line_width{rr});
        end
        hh_v{rr} = hh;
    end
    if uu >= 2
        for kr = 1:length(new_rob_traj)
            plot(new_rob_traj{kr}(1,uu-1:uu),new_rob_traj{kr}(2,uu-1:uu),'Color',color_transparency{kr},'LineWidth',2,'LineStyle','-.');
        end
    end
    drawnow;
    pause(0.5);
    % update
    for kr = 1:length(new_rob_traj)
        delete(hh_m{kr});
        delete(hh_v{kr});
    end
end

% save the robot trajectories to save it in the txt file
message = sprintf('%s\n\nSOLUTION - runs of robots: \n',message);
for j = 1 : length(new_Run_cells)
    message = sprintf('%s\nRobot %d: ',message,j);
    temp = new_Run_cells{j};
    for k = 1 : length(temp)-1
        message = sprintf('%s%d,',message,temp(k));
    end
    message = sprintf('%s%d',message,temp(length(temp)));
end
