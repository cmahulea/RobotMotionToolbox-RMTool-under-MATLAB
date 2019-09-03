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

function Tr = rmt_tr_sys_reduce_Tg(Tg)
% reduce Tg w.r.t. robot permutations - states corresponding to same occupied cells (e.g. (q_1, q_2) and (q_2, q_1)) are collapsed into a single state

N_r = size(Tg.st,2);    %number of robots

st_red=sort(Tg.st,2); %to identify states with same occupied positions in Tg (set of reduced states)
%[st,ind_st,~]=unique(st_red,'rows'); %first output (st) is identical with st_red(ind_st,:)

Tr.st = unique(st_red,'rows');  %reduced states set
st_red_no=size(Tr.st,1);
Tr.Q=1:st_red_no;   %states of Tr
Tr.Obs=Tg.Obs;  %same set of possible observations

Tr.pr=sparse(st_red_no,size(Tr.Obs,1));  %probability of observing a conjunction of props (and only those props) in a current state of Tr (tuple)
Tr.adj=sparse(st_red_no,st_red_no); %adjacency matrix; self-loops will be  added later (otherwise we get out of memory)

for K=1:st_red_no
    tuple = Tr.st(K,:);
    init_states = find(ismember(st_red,tuple,'rows'));   %indices of states of Tg that were collapsed in current tuple
    next_init_states = zeros(1,0);  %will add indices of states in which the ones of Tg transit (row vector)
    for i = 1:length(init_states)
        next_init_states = union(next_init_states,find(Tg.adj(init_states(i),:)));
    end
    
    next_Tr_states = unique(st_red(next_init_states,:) , 'rows'); %next states of Tr
    [~,ind] = ismember(next_Tr_states,Tr.st,'rows');    %indices of next states of Tr
    for i = 1:length(ind)
        Tr.adj(K,ind(i)) = 1+sum(tuple~=Tr.st(ind(i),:));
    end
    
    Tr.pr(K,:) = Tg.pr(init_states(1),:);   %same row with any row for probabilities in init_states (from Tg)
end
 
[~, ind_init_st] = ismember(st_red(Tg.q0,:), Tr.st, 'rows');   %index of initial state of Tr
Tr.q0=ind_init_st;
