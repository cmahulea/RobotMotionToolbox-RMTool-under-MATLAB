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

function [new_run_cells, pos_rob, message2] = rmt_path_planning_dyn_release_resources(Run_cells, rob_traj, data, common, message)
%%dynamical release of resources
%input data:
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

% initial figures (empty environment)
% name_fig = 'InitTrajBoolSpec.fig';
% init_fig = openfig(name_fig)';

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

% initialize the output variables
new_run_cells = ones(size(Run_cells));
new_run_cells(1,:) = Run_cells(idx_end_val(2,1),:);
pos_rob = cell(size(rob_traj));
pos_rob{1} = rob_traj{idx_end_val(2,1)};
size_traj = size(pos_rob{1});


%%common cells for all trajectories
freq_cells = zeros(1,size(data.Pre,1));
for rr = 1:length(rob_traj)
    temp = unique(Run_cells(rr,:));
    freq_cells(temp) = freq_cells(temp) + 1;
end

common_all = find(freq_cells == length(rob_traj));
switch common
    case 'pairs'    %%common cells for trajectory pairs (r_i, r_i+1)
        flag = 0;
    case 'all'
        common_cells = common_all;
        flag = 1;
        if ~isempty(common_cells)
            message2 = sprintf('%s\n The common cells for all trajectories are: ',message2);
            for i = 1:length(common_cells)
                message2 = sprintf('%s %d ', message2, common_cells(i));
            end
        else
            flag = 0;
            message2 = sprintf('%s\n\n There is no common cell among all trajectories. The common cells will be computed for each pair r_i, r_i+1\n\n', message2);
        end
end



for rr = 2:length(rob_traj)
    temp_idx = [];
    temp_idx2 = [];
    prev_rob = idx_end_val(2,rr-1);
    current_rob = idx_end_val(2,rr);

    if flag == 0
        common_pairs = intersect(Run_cells(prev_rob,:), Run_cells(current_rob,:));
        common_cells = common_pairs;
        message2 = sprintf('%s\n The common cells for paths r%d and r%d are: ',message2, prev_rob, current_rob);
        for i = 1:length(common_cells)
            message2 = sprintf('%s %d ', message2, common_cells(i));
        end
    end
    if ~isempty(common_cells)
        pos_rob{rr} = rob_traj{current_rob}(:,end).*ones(size_traj);
        new_run_cells(rr,:) = Run_cells(current_rob,end).*new_run_cells(rr,:);

        for ii = 1:length(common_cells) % save index of the common cells between traj i and i + 1
            temp_idx = [temp_idx find(new_run_cells(rr-1,:) == common_cells(ii),1,'last')]; %index in the previous trajectory, in the new variable
            temp_idx2 = [temp_idx2 find(Run_cells(current_rob,:) == common_cells(ii),1,'first')]; % index in the current trajectory, in the old variable

        end
        temp_idx_first = min(temp_idx);
        temp_length = length(temp_idx);
        temp_idx_first2 = min(temp_idx2);

        new_run_cells(rr,1:temp_idx_first) = Run_cells(current_rob,1:temp_idx_first);
        pos_rob{rr}(:,1:temp_idx_first) = rob_traj{current_rob}(:,1:temp_idx_first);

        smt = [temp_idx_first:temp_idx_first + temp_length-1];
        new_run_cells(rr,smt) = new_run_cells(rr,temp_idx_first);
        pos_rob{rr}(:,smt) = repmat(pos_rob{rr}(:,temp_idx_first),1,length(smt));

        % robot stays in the idle mode until the common regions are crossed by
        % the previous robot
        intermidiate_vector_cells = Run_cells(current_rob,temp_idx_first:temp_idx_first2-1);
        intermediate_vector_pos = rob_traj{current_rob}(:,temp_idx_first:temp_idx_first2-1);

        % eliminate the duplicates, when the robot has more than one idle state
        for j = length(intermidiate_vector_cells):-1:2
            if (isempty(setxor(intermidiate_vector_cells(j),intermidiate_vector_cells(j-1))))
                intermidiate_vector_cells(j) = [];
                intermediate_vector_pos(:,j) = [];
            end
        end
        if ~isempty(intermediate_vector_pos)
            new_run_cells(rr,smt(end)+1:smt(end) + length(intermidiate_vector_cells)) = intermidiate_vector_cells;
            pos_rob{rr}(:,smt(end)+1:smt(end) + length(intermidiate_vector_cells)) = intermediate_vector_pos;
        end
        dif2 = idx_end_val(1,rr) - temp_idx_first2;
        new_run_cells(rr,smt(end) + length(intermidiate_vector_cells) +1:smt(end) + 1+ length(intermidiate_vector_cells) + dif2) = Run_cells(current_rob,temp_idx_first2:idx_end_val(1,rr));
        pos_rob{rr}(:,smt(end) + length(intermidiate_vector_cells) +1:smt(end) + 1+ length(intermidiate_vector_cells) + dif2) = rob_traj{current_rob}(:,temp_idx_first2:idx_end_val(1,rr));
else
        pos_rob{rr} = rob_traj{current_rob};
        new_run_cells(rr,:) = Run_cells(current_rob,:);
    end

end

% delete duplicate destination cell based on the slowest robot
crop_traj = find(new_run_cells(end,:) == new_run_cells(end,end),1,'first');
for ii = size(pos_rob{rr},2):-1:crop_traj+2
    new_run_cells(:,ii-1) = [];
    for rr = 1:length(pos_rob)
        pos_rob{rr}(:,ii) = [];
    end
end

% initial figures (empty environment)
% init_fig = openfig('InitTrajBoolSpec.fig')';
% pause


% concurent movement of the robots
for uu = 1:size(pos_rob{1},2)-1
    for rr = 1:length(pos_rob)
        plot(pos_rob{rr}(1,uu:uu+1),pos_rob{rr}(2,uu:uu+1),'Color',data.rob_plot.line_color{rr},'LineWidth',data.rob_plot.line_width{rr});
        if uu == 1
            plot(pos_rob{rr}(1,uu),pos_rob{rr}(2,uu),'Color',data.rob_plot.line_color{rr},...
                'Marker',data.rob_plot.marker{rr},'LineWidth',data.rob_plot.line_width{rr});
        elseif uu == size(pos_rob{1},2)-1
            plot(pos_rob{rr}(1,end),pos_rob{rr}(2,end),'Color',data.rob_plot.line_color{rr},...
                'Marker',data.rob_plot.marker{rr},'LineWidth',data.rob_plot.line_width{rr},'Color','r');
        end
    end

    pause(0.5);
end

message2 = sprintf('%s\n\nSOLUTION - runs of robots: \n',message2);
for j = 1 : size(new_run_cells,1)
    message2 = sprintf('%s\nRobot %d: ',message2,j);
    temp = new_run_cells(j,:);
    for k = 1 : length(temp)-1
        message2 = sprintf('%s%d,',message2,temp(k));
    end
    message2 = sprintf('%s%d',message2,temp(length(temp)));
end
