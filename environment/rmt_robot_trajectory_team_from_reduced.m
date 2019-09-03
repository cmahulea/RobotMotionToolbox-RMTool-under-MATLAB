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
%   First version released on September, 2014. 
%   Last modification December 29, 2015.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function [R_runs,R_paths,R_trajs,active_robots,path_B] = rmt_robot_trajectory_team_from_reduced(T,T_init,R0_init,Tg,run_Tg,path_Tg,path_B_r,varargin)
%adapt runs from reduced product system Tg (T_T) to robot runs in initial partition
%if no varargin is specified, each robot starts from centroid of its initial cell
%otherwise, varargin{1} is a cell, with element r being a column vector containing starting position of robot r

% R_runs_r={Tg.st(run_Tg{1},:)' , Tg.st(run_Tg{2},:)'}; %project run of Tg to individual and synchronized robot runs
R_paths_r=Tg.st(path_Tg,:)';

n_r=size(R_paths_r,1);    %number of robots

%%adapt R_paths_r to initial partition
R_paths=R0_init';   %column vector
path_B=path_B_r(1); %path_B is made longer (while team is in teh same collapsed regions, B doesn't change state
for i=2:size(R_paths_r,2)
    %T.Cells{R_paths_r(:,i)} - team states to be reached in collapsed system, T.Cells{R_paths_r(:,i)} - current (initial) team deployment in collapsed model
    %R_paths(:,end) - current positions of robots in full system T_init
    R_local_paths = rmt_reduced2full_one_step( T_init, T, R_paths(:,end), R_paths_r(:,i-1) , R_paths_r(:,i) );

%     if size(R_local_paths,2) > 1 %otherwise all robots stay and R_paths remains the same
    R_paths = [R_paths , R_local_paths(:,2:end)];   %first robot positions were already included
%     end
    
    path_B = [path_B, repmat( path_B_r(i) , [1,(size(R_local_paths,2)-1)] )];   %duplicate state in path_B
    
    if i==length(run_Tg{1})+1
        pref_len=size(R_paths,2)-1;  %to split after this R_paths into R_runs
    end
end

path_B = [path_B, path_B_r(end)];

R_runs={R_paths(:,1:pref_len) , R_paths(: , (pref_len+1):end )}; %individual and synchronized robot runs

T=T_init;   %use full partition (with vertices) in the following


%%from here the initial function (for full models) is used (robot_trajectory_team.m);
%synchronizations occur at every transition in partition (althouh they can be reduced only when changing state in teh collapsed model)
n_r=size(R_paths,1);    %number of robots
R_trajs=cell(1,n_r);    %each R_trajs{r} will be a matrix with 2 rows showing trajectory of robot r
active_robots=zeros(n_r,size(R_paths,2));   %active_robots(r,i) is 1 if robot r is changing simplex between position i and i+1 from R_paths; last column dependson suffix

for r=1:n_r
    R_trajs{r}=zeros(2,size(R_paths,2)+1);
    if size(varargin,2)==0  %no initial position specified
        R_trajs{r}(:,1)=mean(T.Vert{R_paths(r,1)},2);    %initial position of robot r: centroid of initial triangle
    else
        R_trajs{r}(:,1)=varargin{1}{r}; %vector giving position
    end

    for i=2:size(R_paths,2) %add succesive positions of robot r (middle points of segments)
        %current simplex of robot r is R_paths(r,i-1)
        %a robot waiting in the same simplex should move to the boundary with the next simplex (first position in run different than the actual simplex, if any)
        %find the next simplex:
        next_simplex=[];
        for j=i:size(R_paths,2)
            if R_paths(r,i-1)~=R_paths(r,j)   %found different simplex
                if j==i %first simplex is different, robot is actively moving (not just towards waiting to synchronize)
                    active_robots(r,i-1)=1;
                end     %else, active_rob remains zero
                next_simplex=R_paths(r,j);
                break;
            end
        end
        if isempty(next_simplex)    %robot stays in the same simplex until end
            R_trajs{r}(:,i) = mean(T.Vert{R_paths(r,i-1)},2);    %converge to centroid
        else
            R_trajs{r}(:,i) = [T.mid_X(R_paths(r,i-1),next_simplex) ; T.mid_Y(R_paths(r,i-1),next_simplex)];  %middle point of segment shared by two successive states (robot r entering j^th cell)
        end
        
        %columns in R_trajs{r} may be succesively repeated if synchronization is needed
%         %this would be for waiting in centroid:
%         if R_paths(r,i-1)~=R_paths(r,i) %robot r changes cell at i^th state from its run
%             R_trajs{r}(:,i) = [T.mid_X(R_paths(r,i-1),R_paths(r,i)) ; T.mid_Y(R_paths(r,i-1),R_paths(r,i))];  %middle point of segment shared by two successive states (robot r entering i^th cell)
%         else
%             R_trajs{r}(:,i) = mean(T.Vert{R_paths(r,i)},2);    %converge to centroid instead of waiting at cell border
%         end
    end

    %after end of suffix (last column in R_trajs{r} is for beginning a new suffix):
    %for robots with last state in suffix equal to first one, just converge to centroid; for others, insert next middle point
    if R_runs{2}(r,end)==R_runs{2}(r,1) %robot r doesn't change state (its suffix may have length 1, but we don't explicitly need this information)
        R_trajs{r}(:,end)=mean(T.Vert{R_runs{2}(r,1)},2);   %converge to centroid of current state
    else    %robot r changes its state
        R_trajs{r}(:,end)= [T.mid_X(R_runs{2}(r,end),R_runs{2}(r,1)) ; T.mid_Y(R_runs{2}(r,end),R_runs{2}(r,1))]; %mid point for starting a new iteration of suffix with robot r
        active_robots(r,end)=1; %active move
    end
    
%     initial_position{r}=R_trajs{r}(:,1);    %initial position
end
