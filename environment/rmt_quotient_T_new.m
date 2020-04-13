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

function T = rmt_quotient_T_new(T)
%abstract T by collapsing states based on observations
%collapsed states have same observation and were reachable in T by going through states with same observation
%each observation of T is a conjunction of one or more propositions, and the index of last observable is free space

for i = 1 : length(T.Q)
    T.Cells{i} = T.Q(i);
end
change=1;
while change
    change = 0;
    for i = length(T.Q):-1:1
        adjacent = setdiff(find(T.adj(i,:)),i);
        for j = 1 : length(adjacent)
            if (T.obs(i) == T.obs(adjacent(j))) %adjacent region with the same observation
                change = 1;
                T.adj(i,:) = T.adj(i,:) + T.adj(adjacent(j),:);
                T.adj(:,i) = T.adj(:,i) + T.adj(:,adjacent(j));
                T.adj(adjacent(j),:) = [];%fuse both regions
                T.adj(:,adjacent(j)) = [];
                T.Q(adjacent(j)) = [];
                T.obs(adjacent(j)) = [];
                for k = 1 : length(T.props)
                    temp = T.props{k};
                    temp = setdiff(temp,adjacent(j));
                    for l = 1 : length(temp)
                        if (temp(l) > adjacent(j))
                            temp(l) = temp(l) - 1;
                        end
                    end
                    T.props{k} = temp;
                end
                T.m0(i) = T.m0(i) + T.m0(adjacent(j));
                T.m0(adjacent(j)) = [];
                for l = 1 : length(T.RO)
                    if (T.RO(l) > adjacent(j))
                        T.RO(l) = T.RO(l) - 1;
                    elseif (T.RO(l) == adjacent(j))
                        T.RO(l) = i;
                    end
                end
                T.Cells{i} = union(T.Cells{i},T.Cells{adjacent(j)});
                T.Cells(adjacent(j)) = [];
                break;
            end %if
        end %for j
        if change
            break;
        end
    end %for i
end %while

T.Vert = [];
T.Q = 1 : length(T.Q);
T.mid_X = [];
T.mid_Y = [];

