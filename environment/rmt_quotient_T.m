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

function Tr = rmt_quotient_T(T)
%abstract T by collapsing states based on observations
%collapsed states have same observation and were reachable in T by going through states with same observation
%each observation of T is a conjunction of one or more propositions, and the index of last observable is free space

Tr.Q=[];
Tr.OBS_set=T.OBS_set;   %same observable set

for i=1:size(T.OBS_set,1)   %for each different observation
    states=find(T.obs==i);  %all states with this observable
    A=sparse(size(T.adj,1),size(T.adj,1));  %adjacency matrix keeping states with current observable and
                                            %transitions between them
    for j=states
        tr_st=intersect(find(T.adj(j,:)),states);   %states with current obs. in which j can transit
        A(j,tr_st)=1;
        A(tr_st,j)=1;
    end
    
    Reach=A;    %Reachable states (in maximum (states-1) transitions)
    for j=2:length(states-1)
        Reach=(Reach+A^j)~=0;
    end
    
    %add for each connected component a state in Tr
    while ~isempty(states)
        Tr.Q(end+1)=length(Tr.Q)+1; %add a state in reduced Tr
        Tr.Cells{Tr.Q(end)}=find(Reach(states(1),:));   %connected component containing first from current "states"
        Tr.obs(Tr.Q(end))=i;    %current observable is i (from for on line 8)
        states=setdiff(states,Tr.Cells{Tr.Q(end)});
    end
end    

Tr.adj=sparse(eye(length(Tr.Q)));   %create adjacencies
for i=Tr.Q  %adjacency from each state of Tr
    neigh_cells=[];   %neighboring cells from partition
    for j=Tr.Cells{i}   %each contained partition cell
        neigh_cells=union(neigh_cells,find(T.adj(j,:))); %Do the union of the first and the adjacent cell
    end
    
    % Riferimento all'automa ridotto
    while ~isempty(neigh_cells)
        for j=Tr.Q
            if ismember(neigh_cells(1),Tr.Cells{j}) %verifica se gli elementi di neigh_cell sono contenuti in Tr.Cells e crea un unico vettore dove gli elementi del primo sono contenuti nel secondo 
                Tr.adj(i,j)=1;  %adjacency
                Tr.adj(j,i)=1;
                neigh_cells=setdiff(neigh_cells,Tr.Cells{j});   %transition to this set already added
                break;  %break for loop, because elements in Tr.Cells are disjoint
            end
        end
    end
end

%addapt Tr.props - states of Tr belonging to each defined region (proposition)
Tr.props=cell(1,length(T.props));  %Tr.props{i} will be row vector with indices of states of 
                                   %Tr included in proposition(region) i
for i=1:length(Tr.props)
    ind_obs=find(sum(Tr.OBS_set==i , 2));  %indices of observables containing prop. i
    Tr.props{i}=find(ismember(Tr.obs,ind_obs)); %searched cells
end

if isfield(T, 'm0') %initial states -> construct initial marking for PN of Tr
    Tr.m0=zeros(length(Tr.Q),1);    %put different than 0 where robots are initially
    for i=Tr.Q
        Tr.m0(i)=sum(ismember(T.R0,Tr.Cells{i}));  %number of robots initially in reduced state i
    end
end
