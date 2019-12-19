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

function [T,propositions] = rmt_tr_sys_partition(regions,x_max,y_max,random_grid)
%regions is a cell with each element a 2-row matrix; x_max,y_max define the boundaries of world (lower-left point is 0,0)
%function creates partition and states of transition systems (not observables/probabilities)

if (random_grid == 0)
    [C,adj,OBS_set,obs,mid_X,mid_Y]=rmt_triangular_decomposition_regions(regions,[0,x_max,0,y_max]);  %triangular decomposition: C is a cell containing vertices of triangles
                                                                    %adj is adjacency (symmetric)
                                                                    %obs is the observation of each cell: obs(i) is the satisfied defined region (1,2,...), and is 0 if cell i lies in the free space
                                                                    %obs is ignored for this function (we'll define propositions as unions of cells (predefined regions))
else
    [C,adj,OBS_set,obs,mid_X,mid_Y]=rmt_grid_decomposition_regions(regions,[0,x_max,0,y_max]);  %triangular decomposition: C is a cell containing vertices of triangles
end    
    
no_states=length(C);
T.Q=1:no_states;  %states
T.Vert=C; %vertices of regions (on rows of C{i})
T.adj=adj;    %transitions based on adjacency (fully actuated robots)
T.OBS_set=OBS_set;
T.obs=obs;
T.mid_X=mid_X;  %the X-coordinate of middle point of adjacent cells i and j is T.mid_X(i,j)
T.mid_Y=mid_Y;

%see what cells are inside each region (use outputs of triangulation
%(rather than modifying inside the triangulation function)
N_p=length(regions);    %number of regions of int.\props
propositions=cell(1,N_p);
for i=1:N_p
    ind_obs = find(sum(OBS_set==i , 2));  %indices of observables containing prop. i
    propositions{i} = find(ismember(obs,ind_obs)); %cells that belong to region i
    T.props{i}=propositions{i}; %searched cells
end
