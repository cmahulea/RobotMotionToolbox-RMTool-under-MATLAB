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

function [Btrans] = rmt_reduce_obs_Buchi(OBS_set,Btrans)
% This function reduce the set of active observations for the
% Buchi automaton, and add also the negated observations,from state k to state l, 
%
%e.g., IF from state k to state l one of this two observations are needed to be active:
%    [1 0 0] and [1 3 0] - lines from OBS_set, AND [0 1 1] and [0 1 0] -
%    are the lines from not_OBS
%     THEN
%    it is reduced the number of active observations, remaining only [1 0
%    0] for active observation and [0 1 0] for inactive observation
% ------
% ** explication: 
%       active observations: line i -> [1 0 0] = y1 & !y2 & !y3   
%             line j -> [1 3 0] = y1 & !y2 & y3
%       inactive observations: line i-> [0 1 1] = !y2 & ! y3
%               line j -> [0 0 1] = !y3
% ====> region y3 can be reduced, because from state k to state l is active
% only one observation -->
% (y1 & !y2 & !y3) | (y1 & !y2 & y3) = y1 & !y2 & (!y3 | y3) = y1 & !y2
%       Btrans{k,l} = [i j] -> i -> corresponds to line i from matrix OBS_set represents the
%       active observation which remains after the reduction
%                           -> j -> corresponds to line j from matrix not_OBS represents the
%       inactive observation which remains after the reduction
% --------
% Note: The decrease of active observation from state k to state l can be
% done ONLY if the two observations differ by only one region of interest
% yi


% inputs:
% OBS_set - the set of observations for the entire team of agents
% Btrans - transitions matrix for Buchi automaton (B.trans)

% output:
% Btrans - modified transitions matrix for Buchi automaton (B.trans), with
% less transitions

temp_Btrans = {};
% temp_Btrans{i,j} will have be a matrix with 2 columns. Each line have the
% captures the number of regions of interest (first column) from the
% observation from OBS_set (second column)
% ** the matrix is sorted based on the number of regions of interest in
% descend order (first line, first column contains the maximum number of
% regions of interest from all the observations)
% e.g., line [3 4] from temp_Btrans{i,j} represent that line 4 from OBS_set
% contains of 3 regions of interest

for i = 1:size(Btrans,1)
    for j = 1:size(Btrans,2)
        submatrix_trans = OBS_set(Btrans{i,j},:); % take all the transitions from B.trans, from state i to state j
        no_nnz = [];
        if ~isempty(submatrix_trans)
            Btrans{i,j} = [Btrans{i,j} Btrans{i,j}];
            for idx_j = 1:size(submatrix_trans,1)
                no_nnz = [no_nnz; nnz(submatrix_trans(idx_j,:))]; % count the number of elements different than 0 ([1 2 0] - this observation contains 2 regions of interest
            end
            no_nnz = [no_nnz Btrans{i,j}(:,1)];
            temp_Btrans{i,j} = sortrows(no_nnz,1,'descend'); %contains the descend order of the number of transitions from the state i to state j, in the Buchi automaton
        end
    end
end

% for all combination from state i to state j from Buchi, take all the observations and eliminate all
% the observations which have contains at least one subset from another observation
% e.g., from observations [1 2 0] and [ 1 2 3], eliminate [1 2 3]
for i = 1:size(temp_Btrans,1) % take all transitions from state i to state j
    for j = 1:size(temp_Btrans,2)
        if ~isempty(temp_Btrans{i,j}) &&...
                (size(temp_Btrans{i,j},1) ~= size(OBS_set,1)) && (size(temp_Btrans{i,j},1) ~= 1)
            for idx_i = 1:length(find(temp_Btrans{i,j}(:,1) > 1)) % take all the observations from state i to state j
                big_set_size = temp_Btrans{i,j}(idx_i,1);
                for idx_j = (idx_i+1):size(temp_Btrans{i,j},1)
                    small_set_size = temp_Btrans{i,j}(idx_j,1);
                    aux_dif = setdiff(OBS_set(temp_Btrans{i,j}(idx_i,2),:), OBS_set(temp_Btrans{i,j}(idx_j,2),:));
                    k = find(Btrans{i,j}(:,1) == temp_Btrans{i,j}(idx_i,2));
                    if (small_set_size == big_set_size - 1) &&... %check if the difference in the size of the 2 observation is equal with 1
                            (length(aux_dif) == 1) && ~isempty(k)  
                        true_obs = Btrans{i,j}(:,1);  % keep on the first column the index of the active observation that remains after the reduction - the smaller observation remains(this index coresponds to the same line from OBS_set)
                        true_obs(k) = [];
                        not_obs = Btrans{i,j}(:,2);  % keep on the second column the index of the inactive observation that remains after the reduction - this index corresponds to the deleted active observation (this index coresponds to the same line from not_obs)
                        not_obs(k) = [];
                        not_obs(k) = temp_Btrans{i,j}(idx_i,2);
                        Btrans{i,j} = [true_obs not_obs];
                    end
                end
            end
            temp_Btrans{i,j}(idx_i,:) = [];

        end
    end
end