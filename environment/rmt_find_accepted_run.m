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

function [run_T,run_B,run_P,path_T,path_B] = rmt_find_accepted_run(P)
%return also run of P and of B (useful because observables of T are probabilistic, and we check is indeed the desired runs are followed in B (and P))
%transitions can have different costs, not only adjacency is captured (as in an old function with same name)
%find an accepted run of product automaton P and project it to states of T (we need a run for T satisfying the LTL formula)
%(see definition of an accepted run: start from an initial state and visit infinitely often the set of final states)
%run_T will be a cell array with 2 elements: first (prefix) is a vector starting from initial state and containing a run (up to a neighbor of a final state, fs),
%second (suffix) is a vector containing a path which must be repeated infinitely (it starts with fs - and it ends with a neighbor of fs)
%(if the suffix has just an element (fs), then fs has a loop in itself, which will be repeated infinitely often)
%the returned run_T will be the run with the smallest cost (shortest) prefix and smallest cost (shortest) suffix for that prefix

run_P=cell(1,length(P.S0)); %initialize with empty runs for each initial state of P; each element will be either empty or a cell with 2 elements (prefix & suffix)
r_cost=Inf(1,length(P.S0)); %cost of prefix+suffix for each run
for i=1:length(P.S0)    %find shortest run for P (if possible) for each initial state (P can have more initial states-same no. as Buchi; T has only one dummy init state, see autom_prod): find prefix and sufix for P
    [prefix, p_cost] = find_paths(P.trans, P.S0(i), P.F);   %find paths from each initial state of P to all final states
    %prefix = prefix( find( ~cellfun('isempty',prefix)));    %elimintate empty prefixes
    empt=find(p_cost==Inf); %indices of infinite costs (corresponding to empty prefixes) (above line worked if we don't take into account the cost)
    prefix(empt)=[];    %remove empty prefixes and their corresponding costs
    p_cost(empt)=[];
    if ~isempty(prefix) %there exists at least one non-empty prefix
        %[sh_p ind] = sort( cellfun('length',prefix));    %we need the order from shortest to longest prefix (we don't need sorted vector sh_p)
        [sh_p ind]=sort(p_cost);    %ascending order of costs for prefixes
        for j=ind   %scan prefixes from shortest one and search for corresponding suffixes
            suffix=[];  %empty vector (useful to test if a suffix was found)
            if P.trans(prefix{j}(end),prefix{j}(end))   %final state from this prefix has a loop to itself
                suffix=prefix{j}(end);
                sh_s=1; %suffix is a self-loop, associate minimum cost 1
            else
                neigh=find( P.trans(:, prefix{j}(end) ));    %states which can transit in final state of current prefix
                                                        %(we cannot use Dijkstra algorithm blindly from fs to fs, because we would get path fs, even if fs has no loop)
                if ~isempty(neigh)   %there is at least a state transiting in fs of current prefix
                    [suffix, s_cost] = find_paths(P.trans, prefix{j}(end), neigh);    %paths to each neighbor which can transit in fs
                    s_cost=s_cost+full(P.trans(neigh,prefix{j}(end)))'; %update costs, by adding costs from neighbors to the current final state
                    %suffix = suffix( find( ~cellfun('isempty',suffix)));    %elimintate empty suffixes
                    empt=find(s_cost==Inf); %eliminate empty suffixes and their costs
                    suffix(empt)=[];
                    s_cost(empt)=[];
                    if ~isempty(suffix)
                        %[sh_s ind_suf] = min( cellfun('length',suffix));    %shortest suffix (if there are more with equal length, choose the first one)
                        [sh_s ind_suf]=min(s_cost); %index of cheapest suffix (only first index is returned, if there are more with same cost)
                        %suffix=suffix{ind_suf}([2:end 1]);  %move the final state of suffix to last position; suffix is now a vector
                        suffix=suffix{ind_suf}; %first state from suffix is a final state (the one that is last state from prefix, but it will be removed from there)
                    end
                end
            end
            if ~isempty(suffix) %a suffix was found
                run_P{i}={prefix{j} suffix};    %cell containing run of P for current initial state
                r_cost(i)=p_cost(j)+sh_s;   %cost of prefix+cost of suffix; can modify here for different weights of prefix/suffix costs, e.g. leave only sh_s to consider only suffix cost, etc.
                break   %continue with next initial state
            end
        end
    end
end

%we have runs of P from each initial state (if they exist); we choose the shortest one and project it to states of T
%run_P=run_P( find( ~cellfun('isempty',run_P))); %remove empty runs
empt=find(r_cost==Inf); %remove empty runs
run_P(empt)=[];
r_cost(empt)=[];
if isempty(run_P)
    run_T=[];   %return an empty run of T (meaning that there is no run of T s.t. LTL formula is satisfied)
    run_B=[];
    run_P=[];
    return
end
% %there is at least one non-empty run of P, we want to find the one with shortest sum prefix+suffix
% for i=1:length(run_P)   %for each non-empty run of P (if there is none, the above defined empty run for T is returned)
%     run_length(i)=sum(cellfun('prodofsize',run_P{i}));  %prefix + suffix (to find minimum)
% end
% [r_l ind]=min(run_length);  %we need the index of run with minimum sum prefix+suffix (all runs are infinite, but this one is easiest to control)
[r_l ind]=min(r_cost);  %index of run with minimum cost for prefix+suffix
% run_T={P.S(run_P{ind}{1}(1:end-1),1)' , P.S(run_P{ind}{2},1)'};   %projection of prefix and suffix on states of T
%                                                                  %(states of T are given by the first column of matrix P.S)
%                                                                  %do not remove first state from prefix, because it wasn't a dummy start state of T (see product_autom)
%                                                                  %remove last state from prefix, because it is the first state in suffix

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
