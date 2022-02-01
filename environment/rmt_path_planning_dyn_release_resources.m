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

% % % function [new_run_cells, pos_rob, message2] = rmt_path_planning_dyn_release_resources(Run_cells, rob_traj, data, common, message)
% % % %%dynamical release of resources
% % % %input data:
% % % %   Run_cells - cells crossed by each robot
% % % %   rob_traj - positions of robots thoughout the paths
% % % %   common - string variable: 'all' - considers the common cells for all
% % % %   trajectories/ 'pairs' - considers the common cells between 2 adjacent
% % % %   trajectories (r_i, r_i+1)
% % % % output data:
% % % %   new_run_cells - new variable which stores the running cells for all
% % % %   trajectories, with few idle states (one idle state is represented by
% % % %   the waiting of one robot until the previous robot crosses the common
% % % %   cells
% % % %   pos_rob - new variable which stores the position of the robots in the
% % % %   trajectories, based on the considered idle states
% % %
% % % % initial figures (empty environment)
% % % % name_fig = 'InitTrajBoolSpec.fig';
% % % % init_fig = openfig(name_fig)';
% % %
% % % % find the orer of the robots
% % % idx_end_val = zeros(1,length(rob_traj));
% % % for r = 1:length(rob_traj)
% % %     end_value = Run_cells(r,end);
% % %     idx_end_val(1,r) = find(Run_cells(r,:) == end_value, 1,'first');
% % % end
% % %
% % % [idx_end_val, nr_rob] = sort(idx_end_val(1,:));
% % % idx_end_val = [idx_end_val; nr_rob];
% % %
% % % message2 = sprintf('%s===========\n The order of the robots is: ', message);
% % % for i = 1:size(idx_end_val,2)
% % %     message2 = sprintf('%s %d ', message2, idx_end_val(2,i));
% % % end
% % %
% % % % initialize the output variables
% % % new_run_cells = ones(size(Run_cells));
% % % new_run_cells(1,:) = Run_cells(idx_end_val(2,1),:);
% % % pos_rob = cell(size(rob_traj));
% % % pos_rob{1} = rob_traj{idx_end_val(2,1)};
% % % size_traj = size(pos_rob{1});
% % %
% % %
% % % %%common cells for all trajectories
% % % freq_cells = zeros(1,size(data.Pre,1));
% % % for rr = 1:length(rob_traj)
% % %     temp = unique(Run_cells(rr,:));
% % %     freq_cells(temp) = freq_cells(temp) + 1;
% % % end
% % %
% % % common_all = find(freq_cells == length(rob_traj));
% % % switch common
% % %     case 'pairs'    %%common cells for trajectory pairs (r_i, r_i+1)
% % %         flag = 0;
% % %     case 'all'
% % %         common_cells = common_all;
% % %         flag = 1;
% % %         if ~isempty(common_cells)
% % %             message2 = sprintf('%s\n The common cells for all trajectories are: ',message2);
% % %             for i = 1:length(common_cells)
% % %                 message2 = sprintf('%s %d ', message2, common_cells(i));
% % %             end
% % %         else
% % %             flag = 0;
% % %             message2 = sprintf('%s\n\n There is no common cell among all trajectories. The common cells will be computed for each pair r_i, r_i+1\n\n', message2);
% % %         end
% % % end
% % %
% % %
% % %
% % % for rr = 2:length(rob_traj)
% % %     temp_idx = [];
% % %     temp_idx2 = [];
% % %     prev_rob = idx_end_val(2,rr-1);
% % %     current_rob = idx_end_val(2,rr);
% % %
% % %     if flag == 0
% % %         common_pairs = intersect(Run_cells(prev_rob,:), Run_cells(current_rob,:));
% % %         common_cells = common_pairs;
% % %         message2 = sprintf('%s\n The common cells for paths r%d and r%d are: ',message2, prev_rob, current_rob);
% % %         for i = 1:length(common_cells)
% % %             message2 = sprintf('%s %d ', message2, common_cells(i));
% % %         end
% % %     end
% % %     if ~isempty(common_cells)
% % %         pos_rob{rr} = rob_traj{current_rob}(:,end).*ones(size_traj);
% % %         new_run_cells(rr,:) = Run_cells(current_rob,end).*new_run_cells(rr,:);
% % %
% % %         for ii = 1:length(common_cells) % save index of the common cells between traj i and i + 1
% % %             temp_idx = [temp_idx find(new_run_cells(rr-1,:) == common_cells(ii),1,'last')]; %index in the previous trajectory, in the new variable
% % %             temp_idx2 = [temp_idx2 find(Run_cells(current_rob,:) == common_cells(ii),1,'first')]; % index in the current trajectory, in the old variable
% % %
% % %         end
% % %         temp_idx_first = min(temp_idx);
% % %         temp_length = length(temp_idx);
% % %         temp_idx_first2 = min(temp_idx2);
% % %
% % %         new_run_cells(rr,1:temp_idx_first) = Run_cells(current_rob,1:temp_idx_first);
% % %         pos_rob{rr}(:,1:temp_idx_first) = rob_traj{current_rob}(:,1:temp_idx_first);
% % %
% % %         smt = [temp_idx_first:temp_idx_first + temp_length-1];
% % %         new_run_cells(rr,smt) = new_run_cells(rr,temp_idx_first);
% % %         pos_rob{rr}(:,smt) = repmat(pos_rob{rr}(:,temp_idx_first),1,length(smt));
% % %
% % %         % robot stays in the idle mode until the common regions are crossed by
% % %         % the previous robot
% % %         intermidiate_vector_cells = Run_cells(current_rob,temp_idx_first:temp_idx_first2-1);
% % %         intermediate_vector_pos = rob_traj{current_rob}(:,temp_idx_first:temp_idx_first2-1);
% % %
% % %         % eliminate the duplicates, when the robot has more than one idle state
% % %         for j = length(intermidiate_vector_cells):-1:2
% % %             if (isempty(setxor(intermidiate_vector_cells(j),intermidiate_vector_cells(j-1))))
% % %                 intermidiate_vector_cells(j) = [];
% % %                 intermediate_vector_pos(:,j) = [];
% % %             end
% % %         end
% % %         if ~isempty(intermediate_vector_pos)
% % %             new_run_cells(rr,smt(end)+1:smt(end) + length(intermidiate_vector_cells)) = intermidiate_vector_cells;
% % %             pos_rob{rr}(:,smt(end)+1:smt(end) + length(intermidiate_vector_cells)) = intermediate_vector_pos;
% % %         end
% % %         dif2 = idx_end_val(1,rr) - temp_idx_first2;
% % %         new_run_cells(rr,smt(end) + length(intermidiate_vector_cells) +1:smt(end) + 1+ length(intermidiate_vector_cells) + dif2) = Run_cells(current_rob,temp_idx_first2:idx_end_val(1,rr));
% % %         pos_rob{rr}(:,smt(end) + length(intermidiate_vector_cells) +1:smt(end) + 1+ length(intermidiate_vector_cells) + dif2) = rob_traj{current_rob}(:,temp_idx_first2:idx_end_val(1,rr));
% % % else
% % %         pos_rob{rr} = rob_traj{current_rob};
% % %         new_run_cells(rr,:) = Run_cells(current_rob,:);
% % %     end
% % %
% % % end
% % %
% % % % delete duplicate destination cell based on the slowest robot
% % % crop_traj = find(new_run_cells(end,:) == new_run_cells(end,end),1,'first');
% % % for ii = size(pos_rob{rr},2):-1:crop_traj+2
% % %     new_run_cells(:,ii-1) = [];
% % %     for rr = 1:length(pos_rob)
% % %         pos_rob{rr}(:,ii) = [];
% % %     end
% % % end
% % %
% % % % initial figures (empty environment)
% % % % init_fig = openfig('InitTrajBoolSpec.fig')';
% % % % pause
% % %
% % %
% % % % concurent movement of the robots
% % % for uu = 1:size(pos_rob{1},2)-1
% % %     for rr = 1:length(pos_rob)
% % %         plot(pos_rob{rr}(1,uu:uu+1),pos_rob{rr}(2,uu:uu+1),'Color',data.rob_plot.line_color{rr},'LineWidth',data.rob_plot.line_width{rr});
% % %         if uu == 1
% % %             plot(pos_rob{rr}(1,uu),pos_rob{rr}(2,uu),'Color',data.rob_plot.line_color{rr},...
% % %                 'Marker',data.rob_plot.marker{rr},'LineWidth',data.rob_plot.line_width{rr});
% % %         elseif uu == size(pos_rob{1},2)-1
% % %             plot(pos_rob{rr}(1,end),pos_rob{rr}(2,end),'Color',data.rob_plot.line_color{rr},...
% % %                 'Marker',data.rob_plot.marker{rr},'LineWidth',data.rob_plot.line_width{rr},'Color','r');
% % %         end
% % %     end
% % %
% % %     pause(0.5);
% % % end
% % %
% % % message2 = sprintf('%s\n\nSOLUTION - runs of robots: \n',message2);
% % % for j = 1 : size(new_run_cells,1)
% % %     message2 = sprintf('%s\nRobot %d: ',message2,j);
% % %     temp = new_run_cells(j,:);
% % %     for k = 1 : length(temp)-1
% % %         message2 = sprintf('%s%d,',message2,temp(k));
% % %     end
% % %     message2 = sprintf('%s%d',message2,temp(length(temp)));
% % % end


order_rob = cell(1,size(data.Pre,1));
empty_cells = [];
for i = 1:size(data.Pre,1)
    for j = 1:size(Run_cells,1) % find the order in which the robots cross each cell
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

flag_end_traj = zeros(1,length(unique_Run_cells));
idx_rob_traj = ones(1,length(unique_Run_cells)); % index in the robots trajectory for each robot
length_trajs = [];
k = 1; % initialize the first step

% memorize the final cells for each trajectory
final_cell_traj = zeros(1,length(unique_Run_cells));
for r = 1:length(unique_Run_cells)
    final_cell_traj(r) = unique_Run_cells{r}(end);
end

current_pos_all_rob = [];

while ~isempty(setdiff(flag_end_traj, ones(1,length(unique_Run_cells))))
    for r = 1:length(unique_Run_cells)
        if ~isempty(unique_Run_cells{r}) % the robot advance as long as it still has cells to cross
            new_Run_cells{r}(idx_rob_traj(r)) = unique_Run_cells{r}(1);
            new_rob_traj{r}(:,idx_rob_traj(r)) = temp_rob_traj{r}(:,1);
            current_cell = unique_Run_cells{r}(1); % access the current index in the robot's trajectory
            idx_current_cell = find(order_rob{current_cell} == r); % compute the order of the robot in the current cell
            idx_rob_traj(r) = idx_rob_traj(r) + 1; % increase index in robot's trajectory

            if k == 1
                aux_all_pos = current_pos_all_rob;
            else
                aux_all_pos = current_pos_all_rob(k - 1, :);
            end
            if idx_current_cell == 1 & isempty(find(aux_all_pos == current_cell))
                new_Run_cells{r}(idx_rob_traj(r)) = unique_Run_cells{r}(1); % the robot advance in the next cell
                new_rob_traj{r}(:,idx_rob_traj(r)) = temp_rob_traj{r}(:,1);
%                 order_rob{current_cell}(1) = [];
                unique_Run_cells{r}(1) = [];
                temp_rob_traj{r}(:,1) = [];
            else
                new_Run_cells{r}(idx_rob_traj(r)) = unique_Run_cells{r}(1); % the robot stays in the same cell
                new_rob_traj{r}(:,idx_rob_traj(r)) = temp_rob_traj{r}(:,1);
            end
            current_pos_all_rob(k,r) = new_Run_cells{r}(idx_rob_traj(r));
            %% not goood!!!
% % % %             if isempty(find(aux_all_pos == current_cell))
% % % %                 order_rob{current_cell}(1) = [];
% % % %             end
            if new_Run_cells{r}(idx_rob_traj(r)) == final_cell_traj(r) && flag_end_traj(r) == 0
                flag_end_traj(r) = 1;
                length_trajs = [length_trajs length(new_Run_cells{r})];
            end
        end
    end
    k = k + 1;
end


max_length = max(length_trajs);
aux_traj = [];
for r = 1:length(new_Run_cells)
    if length(new_Run_cells{r}) < max_length
        new_Run_cells{r}(length(new_Run_cells{r})+1:max_length) = new_Run_cells{r}(end);
        aux = length(new_rob_traj{r})+1:max_length;
        new_rob_traj{r}(:,aux) = repmat(new_rob_traj{r}(:,end),1,length(aux));
    end
    aux_traj = [aux_traj; new_Run_cells{r}];
end

% initial figures (empty environment)
name_fig = 'InitTrajBoolSpec.fig';
init_fig = openfig(name_fig)';

% concurent movement of the robots
for uu = 1:max_length-1
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

    pause;
end

% length_trajs = [];
% for r = 1:size(Run_cells,1)
%     checked_cells = [];
%     for t = 1:length(new_Run_cells{r})
%         current_cell = new_Run_cells{r}(t);
%         if length(idx_cell{current_cell}) >= 2 && isempty(find(checked_cells == current_cell)) %at least 2 robots cross the same cell and the cell was not considered yet
%             first_robot = idx_cell{current_cell}(1);
%             if first_robot == r
%                 continue;
%             end % add idle state for robot trajectory only if is not the first one crossing the current cell
%             idx_traj_first = find(new_Run_cells{first_robot} == current_cell,1,'last');
%             idx_current_rob = find(new_Run_cells{r} == current_cell,1,'last');
%
%             % update in the crossed cells of the robots trajectories
%             temp_cells = new_Run_cells{r}(idx_current_rob:end);
%             idx_idle_state = abs(idx_traj_first - idx_current_rob) + find(idx_cell{current_cell} == r) - 1;
%             new_Run_cells{r}(length(new_Run_cells{r})+1:length(new_Run_cells{r}) +1 + length(idx_idle_state)) = 0; % dummy value to extend the length of the trajectories, considering the synchronizations
%             new_Run_cells{r}(idx_current_rob:idx_current_rob + idx_idle_state-1) = new_Run_cells{r}(idx_current_rob-1); % idle state for  the current robot, waiting to cross the current cell from the trajectory
%             new_Run_cells{r}(idx_current_rob + idx_idle_state: idx_current_rob + idx_idle_state + length(temp_cells)-1) = temp_cells;
%
%             % update in the robots positions
%             temp_rob_traj = new_rob_traj{r}(:,idx_current_rob:end);
%             aux = length(new_rob_traj{r})+1:length(new_rob_traj{r}) +1 + length(idx_idle_state);
%             new_rob_traj{r}(:,aux) = repmat([0;0],1,length(aux));
%             aux = idx_current_rob:idx_current_rob + idx_idle_state-1;
%             new_rob_traj{r}(:,aux) = repmat(new_rob_traj{r}(:,idx_current_rob-1),1,length(idx_current_rob:idx_current_rob + idx_idle_state-1));
%             new_rob_traj{r}(:,idx_current_rob + idx_idle_state: idx_current_rob + idx_idle_state + length(temp_rob_traj)-1) = temp_rob_traj;
%             checked_cells = [checked_cells current_cell];
%         end
%     end
%     length_trajs = [length_trajs length(new_Run_cells{r})];
% end
%
% max_length = max(length_trajs) + 1;
% aux_traj = [];
% for r = 1:length(new_Run_cells)
%     if length(new_Run_cells{r}) < max_length
%         new_Run_cells{r}(length(new_Run_cells{r})+1:max_length) = new_Run_cells{r}(end);
%         aux = length(new_rob_traj{r})+1:max_length;
%         new_rob_traj{r}(:,aux) = repmat(new_rob_traj{r}(:,end),1,length(aux));
%     end
%     aux_traj = [aux_traj; new_Run_cells{r}];
% end



% %



