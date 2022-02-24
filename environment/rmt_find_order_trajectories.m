

%    This is part of RMTool - Robot Motion Toolbox, for Matlab 2010b or newer.
%
%    Copyright (C) 2016 RMTool developing team. For people, details and citing
%    information, please see: http://webdiis.unizar.es/RMTool/index.html.
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
function [order_rob_cell,final_cell_traj,new_Run_cells] = rmt_find_order_trajectories(data,Run_cells,No_r)
% order of the robots into cells based on their trajectories
%input: data for total number of cells
%       Run_cells for robot trajectories
%       No_r number of robots
%output: order_rob_cell - for each cell is stored the order in which the
%robots should cross
%       final_cell_traj - memorize the destination cell
%       new_Run_cells - store the initial cell for all robots

order_rob_cell = cell(1,size(data.Pre,1));
for i = 1:size(data.Pre,1)
    for j = 1:No_r % find the order in which the robots cross each cell
        temp_idx = find(Run_cells(j,:) == i,1,'first');
        if ~isempty(temp_idx)
            order_rob_cell{i} = [order_rob_cell{i}; temp_idx j];
        end
    end
    if ~isempty(order_rob_cell{i})
        idx = sortrows(order_rob_cell{i});
        order_rob_cell{i} = idx; % first column represent the index in the robot trajectory, the second column represent the order of the robots through cell i along the trajectory
        order_rob_cell{i}(:,1) = []; % keep only the order of the robots in the current cell i
    end
    
end

% final cell for rob traj
final_cell_traj = Run_cells(:,end);
for r = 1:No_r
    % initial positions for the robots
    new_Run_cells{r}(1) = Run_cells(r,1);
end
end