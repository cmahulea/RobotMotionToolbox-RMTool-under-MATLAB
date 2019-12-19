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
%   First version released on December, 2019. 
%   Last modification December 12, 2019.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function [C,adj,OBS_set,obs,varargout]=rmt_grid_decomposition_regions(objects,env_bounds)
%mk, version: may 2012 (regions/objects may intersect or go outside environment)
%objects is a cell where each element is a matrix with 2 rows, giving the vertices of current region of interest
%OBS_set is a matrix having on rows the indices of satisfied regions, padded with zeros until number of regions
%only feasible conjunctions of regiosn are included in OBS_set, and the last row is for the free )leftover) space
%obs are the observables of each triangle as row indices in OBS_set (OBS_set(obs(k)) contains the regions satisfied by triangle k, padded with zeros)
%env_bounds has the form [x_min,x_max,y_min,y_max]

%first begin creating points (vertices) X
if isempty(objects)
    obs_size = 1;
else
    obs_size = objects{1}(1,2) - objects{1}(1,1);
end

x = (env_bounds(2)-env_bounds(1)) / obs_size;
y = (env_bounds(4)-env_bounds(3)) / obs_size;

C=cell(1,x*y);
adj=sparse(eye(x*y)); %self transitions in every cell

for j = 0 : y-1
    for i = 0 : x-1
        C{j*x+i+1} = obs_size*[i i+1 i+1 i; j j j+1 j+1];
    end
end

OBS_set=[]; %will be a matrix with N_p columns, giving on rows the possible observations. A row like [1 3 0...0] means that current observation is only regions 1 and 3
for i = 1 : length(objects)
    temp = zeros(1,length(objects));
    temp(1) = i;
    OBS_set = [OBS_set; temp];
end
temp = zeros(1,length(objects));
temp(1) = i+1;
OBS_set = [OBS_set; temp];
obs = zeros(1,length(C));
for i = 1 : length(C)
    temp = mean(C{i},2); %centroid of cell C
    region = length(objects)+1; %free space
    for j = 1 : length(objects)
        if (norm(temp-mean(objects{j},2)) < 10^5*eps)
            region = j;
            break;
        end
    end
    obs(i) = region;
end

nr_reg = length(C);
adj=sparse(eye(nr_reg)); %self transitions in every cell

if nargout>4    %if desired, compute middle points between adjacent cells (useful for an angular path finding); avoid reutrn a cell, because of memory usage:
    middle_X=sparse(nr_reg,nr_reg);   %element (i,j) is zero if i=j or if cells i and j are not adjacent, and otherwise it contains X-coordinate for middle segment between i and j
    middle_Y=sparse(nr_reg,nr_reg);
end

for i=1:nr_reg
    for j=i+1:nr_reg %go only through higher indices (adjacency matrix is symmetric)
        if (norm(mean(C{i},2)-mean(C{j},2)) <= 10^10*eps+obs_size) %two common vertices means adjacency
            adj(i,j)=1;
            adj(j,i)=1; %symmetric            
            if nargout>4
                %middle of segment between cells i and j:
                [xi,yi]=polyxpoly([C{i}(1,:) C{i}(1,end)],[C{i}(2,:) C{i}(2,end)],[C{j}(1,:) C{j}(1,end)],[C{j}(2,:) C{j}(2,end)]);
                middle_temp=[mean(xi) mean(yi)];
                middle_X(i,j)=middle_temp(1);
                middle_X(j,i)=middle_temp(1);
                middle_Y(i,j)=middle_temp(2);
                middle_Y(j,i)=middle_temp(2);
            end
        end
    end
end

if nargout>4
    %return middle of adjacent segments:
    varargout(1)={middle_X};
    varargout(2)={middle_Y};
end
