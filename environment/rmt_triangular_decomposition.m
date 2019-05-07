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


function [C,adj,varargout]=rmt_triangular_decomposition(objects,env_bounds)
%mk, version: may 2012 (obstacles may intersect or go outside environment)
%objects is a cell where each element is a matrix with 2 rows, giving the vertices of current obstacle
%env_bounds has the form [x_min,x_max,y_min,y_max]

%first begin creating points (vertices) X
X = [env_bounds(1) env_bounds(3); %space boundary
    env_bounds(2) env_bounds(3);
    env_bounds(2) env_bounds(4);
    env_bounds(1) env_bounds(4)];

%begin creating constraints Cst (indices of lines from X giving linear constraints)
Cst = [1 2; %space boundary
         2 3;
         3 4;
         4 1];

%add obstacle information
for i=1:length(objects)
    ind=size(X,1)+1;    %index for points
    X=[ X; [objects{i}]' ];
    for j=1:(size(objects{i},2)-1)
        Cst=[ Cst; [ind+j-1 , ind+j ] ];  %object's vertices are arranged in order of parcurging convex hull
    end
    Cst=[ Cst; [ind+j , ind ] ];  %close current object
end

%constrained triangulation (Matlab 2010b); warnings may appear to indicate that constraining edges (obstacles) intersect, or some points are repeated
warning('off', 'all');
Triang = DelaunayTri(X, Cst);   %special structure
warning('on', 'all');

X=Triang.X; %new points (vertices) may have appeared, in case of obstacles intersecting
triangles=Triang.Triangulation;  %indices of triangles (points from nex X)

%from the returned triangles, some are inside obstacles (obstacles are triangulated as well);
%we cannot have triangles partially overlapping with obstacles (becasue of Constrained Delaunay);
%however, new points (vertices) may appear - intersection of obstacless

%search indices of traingles from free space
ind=[]; %feasible indices of triangles (row indices in triangles)
for k=1:size(triangles,1)
    ind=[ind,k];    %assume k is good (if not, it will be removed)
    centr=mean(X(triangles(k,:),:));   %centroid of triangle k (if centroid belongs to one or more obstacles, the whole triangle belongs to that/those obstacles)
    if centr(1)<env_bounds(1) || centr(2)<env_bounds(3) || centr(1)>env_bounds(2) || centr(2)>env_bounds(4) %current triangle outside of bounds (possible if obstacles cross outside)
        ind(end)=[];    %remove k from feasible indices
        continue;  %continue with next index k
    end
    for i=1:length(objects) %for each obstacle
        if inpolygon(centr(1),centr(2),objects{i}(1,:),objects{i}(2,:))      %current triangle is inside obstacle i
            ind(end)=[];    %remove k from feasible indices
            break;  %continue with next index k
        end
    end
end

%construct cell C (triangle vertices in each element) and adjacency adj
%return: C- containing cel vertices (cell with each element a matrix with 2 rows and 3 columns)
%    and adj - adjacency matrix, with adj(i,j)=1 if cell i is adjacent to j (adj(i,i)=1 and adj is symmetric)

k=length(ind);  %number of feasible triangles

C=cell(1,k);
adj=sparse(eye(k)); %self transitions in every cell

if nargout>2    %if desired, compute middle points between adjacent cells (useful for an angular path finding); avoid reutrn a cell, because of memory usage:
    middle_X=sparse(k,k);   %element (i,j) is zero if i=j or if cells i and j are not adjacent, and otherwise it contains X-coordinate for middle segment between i and j
    middle_Y=sparse(k,k);
    if nargout>4    %return also common line segments shared by adjacent cells
        com_F=cell(k,k);    %com_F{i,j} is a matrix with vertices of line segment shared by adjacent cells i,j; vertices are on columns
    end
end

for i=1:k
    C{i}=(X(triangles(ind(i),:),:))';    %in matrix triangle, the columns are indices of points from X composing the triangle
    for j=i+1:k %go only through higher indices (adjacency matrix is symmetric)
        common_v=intersect(triangles(ind(i),:), triangles(ind(j),:)); %indices of common vertices (nodes) of cells i and j
        if length(common_v)==2 %two common vertices means adjacency
            adj(i,j)=1;
            adj(j,i)=1; %symmetric
            
            if nargout>2
                %middle of segment between cells i and j:
                middle_temp=mean(X(common_v,:));
                middle_X(i,j)=middle_temp(1);
                middle_X(j,i)=middle_temp(1);
                middle_Y(i,j)=middle_temp(2);
                middle_Y(j,i)=middle_temp(2);
                if nargout>4    %common line segments shared by adjacent cells
                    %com_F{i,j}=X(common_v,:)';  %com_F{i,j} has form [x_i' , x_i'' ; y_i' , y_i'']
                    com_F{i,j}=sortrows(X(common_v,:))';  %com_F{i,j} has form [x_i' , x_i'' ; y_i' , y_i''], x_i'<=x_i''
                    com_F{j,i}=com_F{i,j};
                end
            end
        end
    end
end
% adj=sparse(adj);    %convert to sparse

%arrange vertices of each cell from decomposition to be convex hull
for i=1:length(C)
    ch_in=convhull(C{i}(1,:),C{i}(2,:));
    C{i}=C{i}(:,ch_in(1:length(ch_in)-1));
end

if nargout>2
    %return middle of adjacent segments:
    varargout(1)={middle_X};
    varargout(2)={middle_Y};
    %common line segments shared by adjacent cells
    if nargout>4
        varargout(3)={com_F};
    end
end
