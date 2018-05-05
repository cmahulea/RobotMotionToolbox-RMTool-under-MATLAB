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


function [trajectory, distance] = rmt_optimize_traj_mpc(C,adj,com_F,safe_dist,interm_points,N,...
    x_st,y_st,x_fin,y_fin,varargin)

%%  find start and destination cells
start_cell=0;
destination_cell=0;
for i=1:length(C)   %indices of starting & final cells
    if inpolygon(x_st,y_st,C{i}(1,:),C{i}(2,:))
        start_cell=i;
    end
    if inpolygon(x_fin,y_fin,C{i}(1,:),C{i}(2,:))
        destination_cell=i;
    end
end

if start_cell==0    %start cell not found (either in obstacle, or approximate decomposition like rectangular)
    obstacles=varargin{1};  %if this argument was passed
    for i=1:length(obstacles)
        if inpolygon(x_st,y_st,obstacles{i}(1,:),obstacles{i}(2,:))
            fprintf('\nStart position inside obstacle O_%g.\n',i);
            trajectory=[]; distance=inf; path=[]; cost_path=inf;
            return;
        end
    end
    %not in obstacle, choose as start cell having closest centroid
    min_dist=inf;
    for i=1:length(C)   %indices of starting & final cells
        if norm([x_st;y_st]-mean(C{i},2))<min_dist
            min_dist=norm([x_st;y_st]-mean(C{i},2));
            start_cell=i;
        end
    end
    fprintf('\nStart position not inside a cell. Moved to centroid of cell c_%g.\n',i);
end

if destination_cell==0    %start cell not found (either in obstacle, or approximate decomposition like rectangular)
    obstacles=varargin{1};  %if this argument was passed
    for i=1:length(obstacles)
        if inpolygon(x_fin,y_fin,obstacles{i}(1,:),obstacles{i}(2,:))
            fprintf('\nDestination position inside obstacle O_%g.\n',i);
            trajectory=[]; distance=inf; path=[]; cost_path=inf;
            return;
        end
    end
    %not in obstacle, choose as start cell having closest centroid
    min_dist=inf;
    for i=1:length(C)   %indices of starting & final cells
        if norm([x_fin;y_fin]-mean(C{i},2))<min_dist
            min_dist=norm([x_fin;y_fin]-mean(C{i},2));
            destination_cell=i;
        end
    end
    fprintf('\Destination position not inside a cell. Moved to centroid of cell c_%g.\n',i);
end

old_adj = adj;

%compute the distance between the centroids of the adjacent cells and
%update them in adj

for i = 1 : size(adj,1)-1
    adj(i,i) = 0.0001;
    for j = i+1 : size(adj,1)
        if (adj(i,j) == 1)
            adj(i,j) = norm(mean(C{i},2)-mean(C{j},2)); %distance between the centroids of cells i and j
            adj(j,i) = adj(i,j); %symetric matrix
        end
    end
end

%find the cost from all nodes to the destination
dist = rmt_find_costs_to_destination(adj,destination_cell);

% add the safety distance on each vertex

for i = 1 : size(com_F,1)-1
    for j = i+1 : size(com_F,1)
        if ~isempty(com_F{i,j})
            vertices = com_F{i,j};
            lam = safe_dist / norm(com_F{i,j});
            new_point1 = [(1-lam)*vertices(1,1)+lam*vertices(1,2); (1-lam)*vertices(2,1)+lam*vertices(2,2)]; %first point at a distance d from a vertex
            new_point2 = [lam*vertices(1,1)+(1-lam)*vertices(1,2); lam*vertices(2,1)+(1-lam)*vertices(2,2)]; %second point at a distance d from the other vertex
            new_points(1,:) = linspace(new_point1(1),new_point2(1),2+interm_points); %generate a number of interm_points number of points between previous thwo points
            new_points(2,:) = linspace(new_point1(2),new_point2(2),2+interm_points);            
            com_F{i,j} = new_points;
            com_F{j,i} = new_points;
        end
    end
end


Edges = [x_st ; y_st];
trajectory = [x_st ; y_st];
Cells = [start_cell ; start_cell];
fprintf(1,sprintf('\n Iteration 1'));

while 1
    temp = old_adj^N;
    cells_N = union(find(temp(Cells(1),:)),find(temp(Cells(2),:))); %find all cells reachable from start_cell in N steps
    
    
    for i = 1 : length(cells_N)-1
        for j = i+1 : length(cells_N)
            if ~isempty(com_F{cells_N(i),cells_N(j)})
                Edges = [Edges com_F{cells_N(i),cells_N(j)}];
                Cells = [Cells [cells_N(i)*ones(1,size(com_F{cells_N(i),cells_N(j)},2));...
                    cells_N(j)*ones(1,size(com_F{cells_N(i),cells_N(j)},2))]];
            end
        end
    end
    
    %create a new graph with one node for each point from Edges
    adj_points = zeros(size(Edges,2),size(Edges,2));
    for i = 1 : size(Cells,2)-1
        adj_points(i,i) = 0.001;
        for j = i+1 : size(Cells,2)
            if ~isempty(intersect(Cells(:,i),Cells(:,j))) % check if the points belongs to adjacent cell
                adj_points(i,j) = norm(Edges(:,i)-Edges(:,j));%Euclidean distance between the two points
                adj_points(j,i) = adj_points(i,j);
            end
        end
    end
    
    %%add a new state corresponding to the final destination
    adj_points = [adj_points zeros(size(adj_points,1),1); zeros(1,size(adj_points,1)) 0];
    Cells(1,size(Cells,2)+1)=destination_cell;
    Cells(2,size(Cells,2))=destination_cell;
    Edges(1,size(Edges,2)+1) = x_fin;
    Edges(2,size(Edges,2)) = y_fin;
    
    %add the terminal cost to the destination
    for i = 1 : size(Cells,2)-1
        adj_points(i,size(adj_points,2)) = min( dist(Cells(1,i)) + norm(Edges(:,i)-mean(C{Cells(1,i)},2)), ...
            dist(Cells(2,i)) + norm(Edges(:,i)-mean(C{Cells(2,i)},2)));
        adj_points(size(adj_points,2),i) = adj_points(i,size(adj_points,2));
    end
    
    [path2, ~] = rmt_find_path(adj_points,1,size(adj_points,2));
    
    trajectory = [trajectory Edges(:,path2(2))];
    if ~isempty(intersect(Cells(:,path2(2)),destination_cell)) %arrived at the destination cell
        trajectory = [trajectory zeros(2,1)];
        trajectory(1,size(trajectory,2)) = x_fin;
        trajectory(2,size(trajectory,2)) = y_fin;
        distance = 0;
        for i = 1 : size(trajectory,2)-1
            distance = distance + norm(trajectory(:,i)-trajectory(:,i+1));
        end
        return;
    end
    
    Edges = trajectory(:,size(trajectory,2));
    Cells = Cells(:,path2(2));
    
    fprintf(1,sprintf('\n Iteration %d',size(trajectory,2)));
    
end
%x_st; y_st

