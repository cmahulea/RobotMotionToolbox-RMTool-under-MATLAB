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

function Tr = rmt_tr_sys_team_reduced(T,R0,propositions,probability)
%construct a reduced team model tr. sys w.r.t. robot permutations - states corresponding to same occupied cells (e.g. (q_1, q_2) and (q_2, q_1)) are collapsed into a single state

N_r=length(R0); %N_r is the number of robots in team

temp_cell=mat2cell( repmat(T.Q,N_r,1) , ones(1,N_r) , length(T.Q) );  %duplicate states of transition systems
st_all=rmt_cartesian_product(temp_cell{:}); %all tuples (on rows, possible combinations, including repetitions) - set will be reduced
st_red=sort(st_all,2); %to identify states with same occupied positions (set of reduced states)
%[st,ind_st,~]=unique(st_red,'rows'); %first output (st) is identical with st_red(ind_st,:)

Tr.st = unique(st_red,'rows');  %reduced states set
st_no=size(Tr.st,1);
Tr.Q=1:st_no;   %states of Tr

%construct possible set of observations (dummy proposition will be the last one) - part similar to the one in function "tr_sys_prob_obs" (for 1 robot)
temp_obs=1:length(propositions);   %atomic propositions, for constructing possible observations (without dummy prop)
ind_dummy=length(propositions)+1;   %index of dummy proposition
N_p=length(propositions);   %number of propositions (excepting the dummy one-environment) that can be simultaneously satisfied (take maximum - assume all props. can overlap)
% N_p = min(N_r,length(propositions)); %this can be used only for non-overlapping regs
temp_cell=mat2cell( repmat(temp_obs,N_p,1) , ones(1,N_p) , length(temp_obs) );  %duplicate observables of transition systems
temp_obs=rmt_cartesian_product(temp_cell{:});  %possible observables, on rows (more robots can have the same observables, that's why we use carth product); observables will be labeled with indices of rows (in T.obs)
temp_obs=unique(sort(temp_obs,2),'rows');   %sort rows and remove identical ones (they would have the same observable)

for i=1:size(temp_obs,1)  %modify observables (keep at most one occurence of same prop on each row, and pad with zeros until length 
    obs=unique(temp_obs(i,:));    %keep unique elements on each row, and pad wih zeros
    if length(obs)<size(temp_obs,2) %pad with zeros until number of propositions
        obs((end+1):size(temp_obs,2))=0;
    end  
    temp_obs(i,:)=obs;
end

temp_obs=unique(temp_obs,'rows');   %again remove identical rows (there may appear new repetitions, due to keeping only unique elements on each row and padding with zeros)
%until now temp_obs has 2^n-1 lines (-1 for the empty set); add a line for the dummy prop (we shouldn't add dummy to other satisfied props, only on a single line)
% temp_obs(end+1,:)=[N_p+1 , zeros(1,N_p-1)]; %dummy has index (N_p+1), and pad with zeros after it
temp_obs(end+1,:)=[ind_dummy , zeros(1,N_p-1)]; %dummy has index "ind_dummy", and pad with zeros after it
% temp_obs(end+1,:)=[ind_dummy]; %dummy has index "ind_dummy", and pad with zeros after it

Tr.Obs=temp_obs; %Tr.Obs contains possible observables of the system (on rows, propositions that can be satisfied; last row for dummy)

Tr.pr=sparse(st_no,size(Tr.Obs,1));  %probability of observing a conjunction of props (and only those props) in a current state of Tr (tuple) - same structure and meaning as in T for 1 robot(WODES2012)
Tr.adj=sparse(st_no,st_no); %adjacency matrix; self-loops will be  added later (otherwise we get out of memory)


for K=1:st_no
    tuple_K = Tr.st(K,:);  %N_r-tuple corresponding to state K -> elements are sorted ascendingly
    
    %adjacency and transitions from of state K (no different costs, robots may waste energy):
    Neigh=cell(1,N_r);
    for i=1:N_r   %there are N_r subpolytopes corresponding to state K
        Neigh{i}=find(T.adj(tuple_K(i),:));    %vector with subpolytopes adjacent to i-th subpolytope of K (including self loop i)
    end
    trans_sets=rmt_cartesian_product(Neigh{:});   %matrix with rows containing ordered tuples in which state K (tuple_K) can transit
    
    trans_sets=sort(trans_sets,2); %sort each row ascendingly (as is tuple_K)
    trans_sets=unique(trans_sets,'rows'); %keep unique rows
    
    [~,ind]=ismember(trans_sets,Tr.st, 'rows');   %indices of states in which the current one can transit (vector ind contains also current state)
    
%    Tr.adj(K,ind)=1;  %transitions (self-loops are again included)
    for i=1:length(ind)
        Tr.adj(K,ind(i))=1+sum(tuple_K~=Tr.st(ind(i),:)); %costs on transitions (based on numebr of changed states - but not necessarily how many move)
    end
    
    
    if isempty(setdiff(tuple_K,R0)) && isempty(setdiff(R0,tuple_K)) %true for one tuple
        Tr.q0 = K;  %index of initial state (cannot find it by comparing one time with all rows of Tr.st, because cannot sort R0 - if it has identical cells, they would be collapsed
    end
    
    
    %probabilities of observing a set and only those propisitions in the set of cells occupied by robots
    tuple_K=unique(tuple_K);    %remove identical states (cells)
    possible_props=[];  %vector with indices of propositions that can appear (propositions that include at least one cell from current tuple)
    for p=1:length(propositions)
        if ~isempty(intersect(tuple_K,propositions{p})) %prop p intersects some of the occupied cells
            possible_props=[possible_props, p];
        end
    end
    if isempty(possible_props)  %no proposition can include actual cells (all of them are in left-over space)
%         possible_props=N_p+1;   %dummy proposition (it is not included in proposiitons)
        possible_props=ind_dummy;   %dummy proposition (it is not included in proposiitons)
        Tr.pr(K,size(Tr.Obs,1))=1;    %last observation (row) in T.Obs was the dummy prop, and probability is 1 (environment exists)
        continue;   %continue with next state
    end
    
    for j=1:(size(Tr.Obs,1)-1)    %for each observable (row of T.Obs) different than the last one (dummy prop), compute probability of observing all and only those propositions
         props_obs_T=setdiff(Tr.Obs(j,:) , 0);   %satisfied propositions for current observation of T - remove 0 (which is not dummy, is just for padding rows of T.Obs)
         if sum(ismember(props_obs_T,possible_props)) == length(props_obs_T) %we have relation of sets "props_obs_T" \subseteq "possible_props"
             %only for the above condition probability will be different than zero (otehrwise, there is a prop in props_obs_T that cannot be satisfied at current state)
             non_props=setdiff(possible_props,props_obs_T);    %these propositions should be non-satisfied (not appeared/false)
             if isempty(non_props)  %if set non_props is empty (possible_props is equal to props_obs_T), all props_obs_T should be satisfied 
                 Tr.pr(K,j)=prod(probability(props_obs_T));    %probability of having all "possible_props" (set equal to "props_obs_T")
             else
                 Tr.pr(K,j)=prod(probability(props_obs_T)) * prod(1 - probability(non_props));    %probability of having only props from "props_obs_T", and not from "non_props"
             end
         end
    end
    Tr.pr(K,size(Tr.Obs,1))=prod( 1 - probability(possible_props) );  %last observation of T.Obs is just the dummy; compute probability of not observing anything from possible_props (to remain just dummy)
end

