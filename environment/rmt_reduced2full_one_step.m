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

function R_local_paths = rmt_reduced2full_one_step(T_full, T_r, R_init, Tr_init, Tr_dest)
%compute trajectory in full partition based on initial position in partition (full model) and on reduced (collapsed) state to be reached
%R_init = column vector with states in full model T_full
%Tr_init = column vector with initial state in reduced model Tr corresponding to R_init
%Tr_dest = column vector with states to be reached in reduced model Tr
%elements of R_init, Tr_init, Tr_dest have the same order, on robot indices
%R_local_paths will be a matrix with trajectories in T_full; trajectories of robots may have different lengths
%in R_local_paths all trajectories are made of the same length, by repeating last state from shorter paths

n_r=length(R_init);    %number of robots

%if Tr_init not supplied, it can be found using:
% Tr_init=NaN(n_r,1);
% for r=1:n_r
%     for i=1:length(T_r.Q)
%         if ismember(R_init(r), T_r.Cells{i})
%             Tr_init(r)=i;
%             break;
%         end
%     end
% end

%%adapt transition to initial partition
P=cell(1,n_r);
max_run_len=0;
for r=1:n_r
    A=T_full.adj;
    other_regs=setdiff(T_full.Q , [T_r.Cells{Tr_init(r)} , T_r.Cells{Tr_dest(r)}]);   %remove transitions to/from other reduced regions
    A(other_regs,:)=0;
    A(:,other_regs)=0;
    [paths, costs] = rmt_find_paths(A, R_init(r) , T_r.Cells{Tr_dest(r)} );
    
    [~,ind]=min(costs);
    P{r}=paths{ind};
%     if length(paths{ind})>1
%         P{r}=paths{ind}(2:end);    %path of r^th robot (first cell is already in R_paths)
%         disp(1)
%     else
%         P{r}=paths{ind};    %self-loop
%     end
    max_run_len=max(max_run_len,length(P{r}));   %length of maximum run
end

%%path from this stage have to be put at the same length:
if max_run_len==1 %all robots stay
    R_local_paths = R_init;
else
    for r=1:n_r
        if length(P{r})<max_run_len
            if length(P{r})>1
                last_cell=P{r}(end);
                P{r}(end:(max_run_len-1))=P{r}(end-1);  %repeat the cell before last one (if different, a waiting may be needed when entering last cell)
                P{r}(end+1)=last_cell;
            else
                P{r}((end+1):max_run_len)=P{r}(end); %robot stays
            end
        end
    end
    R_local_paths = cell2mat(P');   %R_init is already included in P cell
end
