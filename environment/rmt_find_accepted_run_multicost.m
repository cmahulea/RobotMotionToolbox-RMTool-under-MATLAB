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

function [run_T,run_B,run_P,path_T,path_B,cost_optim] = rmt_find_accepted_run_multicost(P,varargin)
%use multiple costs for finding best accepted run
%varargin specifies costs, in order they should be used (their importancs), as a cell of strings (e.g. varargin{1} can be string 'prob', varargin{2} 'move', etc)
%each varargin is a string with a name of a field of automaton P

run_T = [];
run_B = [];
run_P = [];
path_T = [];
path_B = [];
cost_optim=0;

n_c=length(varargin);   %number of cost functions considered
cost_mat=cell(1,n_c);  %will contain the cost matrices, no matter if function was called for an automaton with name different than P
for i=1:n_c    %add name of automaton (here is P, don't care about the variable under function was called, can be Pg or else)
    cost_mat{i}=eval(['P.',varargin{i}]); %cost matrix i
end

%return also run of P and of B (useful because observables of T are probabilistic, and we check is indeed the desired runs are followed in B (and P))
%transitions can have different costs, not only adjacency is captured (as in an old function with same name)
%find an accepted run of product automaton P and project it to states of T (we need a run for T satisfying the LTL formula)
%(see definition of an accepted run: start from an initial state and visit infinitely often the set of final states)
%run_T will be a cell array with 2 elements: first (prefix) is a vector starting from initial state and containing a run (up to a neighbor of a final state, fs),
%second (suffix) is a vector containing a path which must be repeated infinitely (it starts with fs - and it ends with a neighbor of fs)
%(if the suffix has just an element (fs), then fs has a loop in itself, which will be repeated infinitely often)
%the returned run_T will be the run with the smallest cost (shortest) prefix and smallest cost (shortest) suffix for that prefix

run_P=cell(1,length(P.S0)); %initialize with empty runs for each initial state of P; each element will be either empty or a cell with 2 elements (prefix & suffix)
r_cost=Inf(length(P.S0),n_c+1); %costs of prefix+suffix for each run, on rows (last values on row for number of transitions)
for i=1:length(P.S0)    %find shortest run for P (if possible) for each initial state (P can have more initial states-same no. as Buchi; T has only one dummy init state, see autom_prod): find prefix and sufix for P
    [prefix, p_cost, adj_mat] = rmt_find_paths_multicost(P.S0(i), P.F, cost_mat{:});   %find paths from each initial state of P to all final states - modified Dijkstra for multiple costs
    
    %store also adjacency matrix (with zeros and ones)
    empt=find( cellfun('isempty',prefix)); %indices of empty prefixes (with at least one infinite cost function)
    prefix(empt)=[];    %remove empty prefixes and their corresponding costs
    p_cost(empt,:)=[];
    if ~isempty(prefix) %there exists at least one non-empty prefix
        %[sh_p ind] = sort( cellfun('length',prefix));    %we need the order from shortest to longest prefix (we don't need sorted vector sh_p)
        [~, ind]=sortrows(p_cost);    %ascending order of costs for prefixes; sort after cost1, then after cost2, ...
        for j=ind'   %scan prefixes from shortest one and search for corresponding suffixes
            suffix=[];  %empty vector (useful to test if a suffix was found)
            if adj_mat(prefix{j}(end),prefix{j}(end))   %final state from this prefix has a loop to itself
                suffix=prefix{j}(end);
                sh_s=zeros(1,n_c+1);    %cost for self-loop (last for number of tranzitions)
                for k=1:n_c
                    sh_s(k)=cost_mat{k}(suffix,suffix);
                end
                sh_s(end)=adj_mat(suffix,suffix);
                
            else
                neigh=find( adj_mat(:, prefix{j}(end) ));    %states which can transit in final state of current prefix
                %(we cannot use Dijkstra algorithm blindly from fs to fs, because we would get path fs, even if fs has no loop)
                if ~isempty(neigh)   %there is at least a state transiting in fs of current prefix
                    [suffix, s_cost] = rmt_find_paths_multicost(prefix{j}(end), neigh, cost_mat{:});    %paths to each neighbor which can transit in fs
                    for k=1:n_c %update costs, by adding costs from neighbors to the current final state
                        s_cost(:,k)=s_cost(:,k)+cost_mat{k}(neigh,prefix{j}(end));
                    end
                    s_cost(:,end)=s_cost(:,end)+adj_mat(neigh,prefix{j}(end));
                    
                    empt=find( cellfun('isempty',suffix)); %eliminate empty suffixes and their costs
                    suffix(empt)=[];
                    s_cost(empt,:)=[];
                    if ~isempty(suffix)
                        [~, ind_suf]=sortrows(s_cost); %index of cheapest suffix (if there are more with same cost1, cost2 is sorted ascendingly)
                        ind_suf=ind_suf(1);
                        sh_s=s_cost(ind_suf,:);    %optimum costs of this suffix
                        suffix=suffix{ind_suf}; %first state from suffix is a final state (the one that is last state from prefix, but it will be removed from there)
                    end
                end
            end
            if ~isempty(suffix) %a suffix was found
                run_P{i}={prefix{j} suffix};    %cell containing run of P for current initial state
                r_cost(i,:)=p_cost(j,:)+sh_s;   %costs of prefix+cost of suffix; can modify here for different weights of prefix/suffix costs, e.g. leave only sh_s to consider only suffix cost, etc.
                break   %continue with next initial state
            end
        end
    end
end




%we have runs of P from each initial state (if they exist); we choose the shortest one and project it to states of T
empt=find( cellfun('isempty',run_P)); %remove empty runs
run_P(empt)=[];
r_cost(empt,:)=[];
if isempty(run_P)
    run_T=[];   %return an empty run of T (meaning that there is no run of T s.t. LTL formula is satisfied)
    run_B=[];
    run_P=[];
    cost_optim=Inf;
    return
end
% %there is at least one non-empty run of P, we want to find the one with shortest sum prefix+suffix
[~, ind]=sortrows(r_cost); %index of run with minimum costs for prefix+suffix
ind=ind(1);

cost_optim=r_cost(ind,:);   %optimum vector with costs
run_P={run_P{ind}{1}(1:end-1) , run_P{ind}{2}}; %remove last state from prefix, because it is the first state in suffix
run_T={P.S(run_P{1},1)' , P.S(run_P{2},1)'};   %projection of prefix and suffix on states of T
run_B={P.S(run_P{1},2)' , P.S(run_P{2},2)'};   %projection of prefix and suffix on states of B

%eliminate possible identical successive states (due to next operators, etc.)
for i=1:2
    j=1;    %parcurge prefix/suffix of run
    while j < length(run_T{i})
        if run_T{i}(j)==run_T{i}(j+1)
            run_T{i}(j+1)=[];    %eliminate duplicate
        else
            j=j+1;  %advance if no duplicate at current position
        end
    end
end
%eliminate last state from prefix if identical with first one from suffix
try %prefix may be empty
    if run_T{1}(end)==run_T{2}(1)
        run_T{1}(end)=[];
    end
end
%eliminate last state from suffix if identical with first one from suffix (if suffix has at least 2 states)
if (length(run_T{2})>=2) && run_T{2}(1)==run_T{2}(end)
    run_T{2}(end)=[];
end

%construct path_T, path_B as strings with repetitions of suffix - if length of suffix is loner than 1, include 2 repetitions, otherwise just one
if length(run_T{2})>1
    path_T=[run_T{1} , run_T{2}, run_T{2}];
    path_B=[run_B{1} , run_B{2}, run_B{2}];
else
    path_T=[run_T{1} , run_T{2}];
    path_B=[run_B{1} , run_B{2}];
end
if length(path_B)==length(path_T)   %equal lengths should be only when prefix is empty
    path_B(end+1)=path_B(end); %path_B should have the length longer than path_T with 1 (to be able to go to the next index (i+1))
end
