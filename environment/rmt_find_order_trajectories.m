

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
function [order_rob_cell,varargout] = rmt_find_order_trajectories(data,Run_cells,No_r,varargin)
% order of the robots into cells based on their trajectories
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


if nargout > 1 && nargin == 4

    rob_traj = data.new_traj.rob_traj;
    varargin{1} = rob_traj;
    
    final_cell_traj = Run_cells(:,end);
    final_rob_traj = zeros(2,No_r);
    prev_final_rob_traj = zeros(2,No_r);
    new_rob_traj = cell(size(rob_traj));

    for r = 1:No_r
        % memorize the final cells for each trajectory
        final_rob_traj(:,r) = rob_traj{r}(:,end);

        % memorize the previous from the end point trajectory
        for i = 1:size(rob_traj{r},2)
            verif = ismember(rob_traj{r}(:,i),rob_traj{r}(:,end));
            if isempty(setdiff(double(verif),ones(2,1)))
                prev_final_rob_traj(:,r) = rob_traj{r}(:,i-1);
                break;
            end
        end

        % initial positions for the robots
        new_rob_traj{r}(:,1) = rob_traj{r}(:,1);
        new_Run_cells{r}(1) = Run_cells(r,1);

    end
    varargout{1} = final_rob_traj;
    varargout{2} = final_cell_traj;
    varargout{3} = prev_final_rob_traj;
    varargout{4} = new_rob_traj;
    varargout{5} = new_Run_cells;
end
end