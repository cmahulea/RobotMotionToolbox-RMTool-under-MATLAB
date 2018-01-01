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

function P = rmt_product_autom_prob_team(T,B)
%product automaton without inputs (for deterministic T), and without addind dummy state in T
%T has probabilities on observations, and these will imply probabilities on transitions of P
%probabilities on transitions of P will be logarithmated - there will be costs on transitions of P (for finding a path with max prob.)

%P is a structure with fields: S (states = T.Q x B.S), S0 (initial state, in set T.Q x B.s0), F (final states, to be visited infinitely often, F = T.Q x B.F),
%trans (transitions, no inputs, sparse matrix, idea as in case of T, but here we'll have costs)
%P.S0 will be found from T.q0

prec=1e3*eps;     %prec is used only for search with Dijkstra, in P.trans
                  %for solving LP, use zero precision; otherwise, errors in LP solve may appear; there are two costs: probability and move:
                  %even if transitions with probability 1 would appear as disconnected in P.prob, they are connected in P.move and in P.trans

P.S=rmt_cartesian_product(T.Q, B.S); %states of P (row i gives on position 1 state from T.Q, on position 2 state of B)
st_no=size(P.S,1); %total # states of P; states of P will be labelled 1,2,...,st_no

P.F = find(ismember(P.S(:,2)', B.F));   %row vector to store indices of final states of P

%probability of transition (first cost of P):
P.trans=sparse(st_no,st_no);    %sparse matrix to keep possible transitions (trans(i,j)=cost (different than 0) if S(i,:) can transit in S(j,:), 0 if no transition)
P.prob=sparse(st_no,st_no);     %copy of P.trans, but without precision (used in LP optimization, while trans is for Dijkstra)
%number of moving robots, plus 1 (second cost of P):
P.move=sparse(st_no,st_no);

for i=1:st_no %search for possible transitions from current state
    tr_q=find(T.adj(P.S(i,1),:));  %labels of states of T in which q_i can transit (if any)
    tr_s=find(~cellfun('isempty',B.trans(P.S(i,2),:))); %indices (states) of B in which s_i can transit (for some predicates(observables))

    for k=1:length(tr_s)    %for each next state of B
%         poss_obs_q_T=find(T.pr(P.S(i,1),:));   %indices of possible observations in current state of T (P.S(i,1))
%         enabl_obs=intersect(poss_obs_q_T , B.trans{P.S(i,2),tr_s(k)});    %set of possible current observations of T that enable transition to tr_s(k) in B
%         if ~isempty(enabl_obs)  %there exists at least such an observable (otherwise, do nothing - no transition)
%             prob_trans_P = full( sum(T.pr(P.S(i,1) , enabl_obs)));  %probability of transitions in P from current state to states with tr_s(k) from B (full for avoiding sparse result)
%               %prob_trans_P is in range (0,1]; if it's 1, -log should not be 0 (disconnected, due to sparse matrix)          
          prob_trans_P = full( sum(T.pr(P.S(i,1) , B.trans{P.S(i,2),tr_s(k)})));  %sum of probabilities of observables (if any) in current state of T that enable transition of B
          %probability of transitions in P from current state to states with tr_s(k) from B (full for avoiding sparse result)

          if prob_trans_P>0 %if prob_trans_P is 0, current transition in P is not possible
%              cost_trans_P = prec-log(prob_trans_P); %this is the cost of outgoing transitions from current state of P to states with tr_s(k) on second position
             cost_trans_P = -log(prob_trans_P);
             
             ind_next_states_P=find(ismember(P.S(:,1)', tr_q) & ismember(P.S(:,2)', tr_s(k)));
             
%              P.prob(i, find(ismember(P.S(:,1)', tr_q) & ismember(P.S(:,2)', tr_s(k))) )=cost_trans_P; %set cost for transition in P
             P.trans(i, ind_next_states_P )=cost_trans_P+prec; %set cost for transition in P, by adding prec (to avoid zero costs)
             P.prob(i, ind_next_states_P )=cost_trans_P; %set cost for transition in P (probability log, without prec)
             
%              %additionally, for self-loops in T (do not test in P, because obs of T is accounted at next transition) make cost in P.trans smaller, to favorize remaining (stopping) in a state if it's possible to satify formula this way
             if ismember(P.S(i,1),tr_q) %usually states of T have self-loops, but check in case not all adjacencies are also transitions
                 P.trans(i, ismember(P.S(:,1)', P.S(i,1)) & ismember(P.S(:,2)', tr_s(k)) ) = cost_trans_P - prec/2; %make cost a little smaller
             end             
             
             %compute moving cost for each transition:
             for j=ind_next_states_P %each next state of P
                 P.move(i,j)=T.adj(P.S(i,1),P.S(j,1));  %inherited from T: number of moving robots plus 1
             end
         end
    end
end

P.S0=find(ismember(P.S(:,1)', T.q0) & ismember(P.S(:,2)', B.S0));    %row vector with indices of initial states of P
