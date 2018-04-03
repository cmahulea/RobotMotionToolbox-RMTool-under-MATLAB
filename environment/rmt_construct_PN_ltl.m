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
%   First version released on February, 2017.
%   Last modification February 28, 2017.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function [Pre,Post,m0,final_states] = rmt_construct_PN_ltl(Pre,Post,m0,props,B,OBS_set)
% In this function the Petri net is created starting from the Buchi automaton.
% In particular, a new place (new line) is added to the existing PRE and POST matrices, for each state presents in the the Buchi automaton.
% The same conceptis valid to add a new transitions (new columns) in the PRE and POST matrices, for each transition presents in the Buchi automaton .
% Finally, the arcs are created between the posts and the transitions, obtaining the complete Petri net.

nplaces = size(Pre,1);

for i = 1 : length(props) %add a new place for each observation
    Pre = [Pre; zeros(1,size(Pre,2))];
    Post = [Post; zeros(1,size(Post,2))];
    places_prop = props{i}; %places in which is observed props{i}
    m0 = [m0 ; sum(m0(places_prop))];
    for j = 1 : length(places_prop)   %add arcs from to input/output transitions where the observation
        Post(size(Post,1),:) = Post(size(Post,1),:) + Post(places_prop(j),:);
        Pre(size(Pre,1),:) = Pre(size(Pre,1),:) + Pre(places_prop(j),:);
    end
end


%add a new place for each state in Buchi
Pre = [Pre; zeros(length(B.S),size(Pre,2))];
Post = [Post ; zeros(length(B.S),size(Post,2))];
temp = length(m0);
m0 = [m0 ; zeros(length(B.S),1)];
m0(temp+B.S0) = 1; %add one token to the initial state of the Buchi
final_states = B.F + temp;

for i = 1 : size(B.trans,1)
    for j = 1 : size(B.trans,2)
        temp = B.trans{i,j};
        for k = 1 : length(temp) %add a transition for each disjunctive term
            Pre = [Pre zeros(size(Pre,1),1)];
            Post = [Post zeros(size(Post,1),1)];
            Pre(nplaces+length(props)+i,size(Pre,2)) = 1; %add an arc from place/state i of the Buchi automaton to the new transition
            Post(nplaces+length(props)+j,size(Post,2)) = 1; %add an arc from the new transition to state j of the Buchi automaton
            observ = OBS_set(temp(k),:);
            % with the follow control we discard from the analysis the free space,
            % infact, the columns of OBS_set only concerning the regions of interest
            for l = 1 : length(observ)
                if ((observ(l) ~= 0) && (observ(l)<=length(props)) )
                    Pre(nplaces + observ(l),size(Pre,2)) = 1; %add a double arc from observation place to the new transition
                    Post(nplaces + observ(l),size(Post,2)) = 1;
                end
            end
        end
    end
end
return
