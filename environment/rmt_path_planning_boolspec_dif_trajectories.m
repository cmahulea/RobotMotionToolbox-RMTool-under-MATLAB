
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

function [out_Run_cells, Runs, message] = rmt_path_planning_boolspec_dif_trajectories(data,xmin,N_r,message)
% interpret xmin to obtain robots trajectories and their final destination
% inputs: data - Pre, Post
%         xmin - solution returned by MILP for re-planning the trajectories
%       N_r - number of robots
%       message - update message with info about run time and number of
%       variables
%outputs: out_Run_Cells - new trajectory for each robot
%       Runs - each line from matrix Runs contains the trajectory cells for
%       each robot

out_Run_cells = cell(N_r,1);
Runs = [];
max_length = [];

Pre = data.relres.Pre;
Post = data.relres.Post;

nplaces = size(Pre,1);
ntrans = size(Pre,2);

tic
for r = 1:N_r
    lefts = 2*nplaces+1;
    step = 2*nplaces + ntrans; % used also for each step
    sigma = xmin(lefts + step*(r-1):step + step*(r-1)); %extract sigma from xmin
    
    left_mi = nplaces + 1;
    right_mi = 2*nplaces;
    mi_f = xmin(left_mi + step*(r-1):right_mi + step*(r-1)); % extract mf for r_i din xmin
    
    m0_i = xmin(1 + step*(r-1):nplaces + step*(r-1)); % extract m0 for r_i din xmin - to see the order of the robots
    
    message = sprintf('%s\n============STEP %d =============\n',message,r);
    message=sprintf('%s\nMarking [ %s ] = %s\n',message,mat2str(find(m0_i>eps*10^5)),mat2str(m0_i(m0_i>eps*10^5)));
    message=sprintf('%s\nMarking [ %s ] = %s\n',message,mat2str(find(mi_f>eps*10^5)),mat2str(mi_f(mi_f>eps*10^5)));
    message = sprintf('%s\nSigma [ %s ] = %s\n',message,mat2str(find(sigma>eps*10^5)),mat2str(sigma(sigma>eps*10^5)));
    
    out_Run_cells{r} = find(m0_i > eps*10^10);
    fired_trans = find(sigma > eps*10^10);
    post_cell = [];
    pre_cell = [];
    
    % find the order of the fired transitions
    k = 1;
    while k <= length(fired_trans)
        pre_cell = find(Pre(:,fired_trans(k))); % fire the transition which enables the robot to move from its initial position    
        if pre_cell == out_Run_cells{r}(end)
            out_Run_cells{r} = [out_Run_cells{r} find(Post(:,fired_trans(k)))]; % move to the next cell
            fired_trans(k) = [];
            k = 1;
        else
            k = k + 1;
        end
        if length(fired_trans) == 1 %& fired_trans(k) == mi_f % for the case when the robot must fire only one transition to reach its final cell
            out_Run_cells{r} = [out_Run_cells{r} find(mi_f>eps*10^10)];
            fired_trans(k) = [];
        end
    end
    
    max_length = [max_length length(out_Run_cells{r})];
    
end
time = toc;
message=sprintf('%s\nTime to compute the new trajectories based on the new re-planned paths: %d \n',message,time);

max_length = max(max_length);
% make all trajectories with the same length, by maintaining the final
% position once the robots arrive
for r = 1:N_r
    out_Run_cells{r}(end:end+max_length - length(out_Run_cells{r})) = out_Run_cells{r}(end);
    Runs = [Runs; out_Run_cells{r}];
end

end