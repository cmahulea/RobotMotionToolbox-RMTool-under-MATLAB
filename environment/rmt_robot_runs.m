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

function [Runs, team_run] = rmt_robot_runs(T,Run_cells)
%Runs{i} will be the run of robot i, without repetitions

%Run_cells=sort(Run_cells,1);    %sort each column

team_run=zeros(size(Run_cells));    %matrix with rows, one row for each robot
team_run(:,1)=Run_cells(:,1);   %initial positions

N_r=size(Run_cells,1);  %robots

for i=2:size(Run_cells,2)
    for j=1:N_r
        Neigh{j}=find(T.adj(team_run(j,i-1) ,:));    %neighbors of j^th previous state, including self-loop
        Neigh{j}=intersect(Run_cells(:,i)',Neigh{j});
        possib(j)=length(Neigh{j}); %how many possibilities
    end
    
    trans=rmt_cartesian_product(Neigh{:});   %robot order is important here
    trans_sorted=sort(trans,2);    %sort rows, for being able to compare with next tuple (the one to be reached)
    
    
    [ignore,ind]=ismember(Run_cells(:,i)',trans_sorted, 'rows');   %ind is the row index in trans corresponding to next position
    if length(ind)>1  %if there are more possible tuples, chose the one to which fewer robots have to move
        trans_sorted=trans_sorted(ind,:); %keep only these tuples (there may be more)
        movmnt = trans_sorted==repmat(run_R(j,:)',length(ind),1); %movements
        [ignore,ind]=min(sum(movmnt,2));    %index of tuple giving less movement (first index, if there are more such tuples)
    end
    team_run(:,i)=trans(ind,:); %next tuple, arranged corresponding to robots
end

%create individual runs (rows from team_run), and remove successive repetiitons in each of them (no synchr. needed for Boolean)
Runs=cell(1,N_r);
for r=1:N_r
    run=team_run(r,:);  %run with repeated states
    orig_indices={};    %start with empty cell
    i=1;
    while i<=length(run)
        j=min(i+1,length(run)); %avoid goind to indices outside run
        while run(i)==run(j)
            j=j+1; %keep increasing j for all successive repetitions of the same state
            if j==length(run)+1
                break;
            end
        end
        j=j-1;  %for all indices between i and j we have the same state (i=j if no successive repetitions for current state)
        orig_indices{end+1}=i:j;
        i=j+1;  %continue with next different state
    end
    
    new_run=zeros(1,length(orig_indices));  %construct new_run
    for i=1:length(orig_indices)
        new_run(i)=run(orig_indices{i}(1));
    end
    
    Runs{r}=new_run;    %run of r^th robot
end
