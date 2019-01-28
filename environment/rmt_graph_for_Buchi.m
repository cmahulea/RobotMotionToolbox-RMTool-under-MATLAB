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
%   First version released on January, 2019.
%   Last modification January 31, 2019.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function adj_B = rmt_graph_for_Buchi(B)
%adjacency matrix from Buchi automaton (for searching minimum cost paths in Buchi)
%take unitary costs

N_s=length(B.S);    %number of states in Buchi
adj_B=Inf*ones(N_s,N_s);
for i=1:N_s
    for j=1:N_s
        if ~isempty(B.trans{i,j})   %there is transition from state i to j
            adj_B(i,j)=1;
        end %otherwise, disconnected
    end
end
