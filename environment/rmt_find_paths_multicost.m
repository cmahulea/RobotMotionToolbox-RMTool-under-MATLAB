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


function [paths, costs, varargout] = find_paths_multicost(s,d,varargin)
% %find optimum paths (shortest path) in a graph with multiple cost functions on edges, from a source node s to several nodes (d), and the corresponding costs
%optimum means that first cost1 is minimized, than among all paths with optimum_cost1 it is choosen one that minimizes cost 2, and among those cost3, and so on
%the path returned is of shortest possible length such that it optimizes all costs in the described manner
%(so, there is no need to specify a last unitary cost function)

%the graph is known through square (possibly sparse) matrices from varargin
%in such a matrix adj(i,j)=cost for going from i to j; exception if adj(i,j)==0 - see below
%an adaptation of Dijkstra's algorithm is used (since it finds minimal costs for all nodes in graph), with the following small modification:
%if adj(i,j)==0, (i~=j), then there is no transition from i to j, so this 0 is not a cost (for i==j, than a self-loop is always considered)
%(this is done because we want that adj be a sparse matrix, for managing more states;
%we will check to not have costs less than precision, which could be approximated with 0)
%s - index for soure node (scalar), d - row vector with indices for destination nodes
%paths will be a cell array, paths{i} will be a vector giving the path s -> d(i)
%costs will be a matrix vector, costs(i,:) gives the costs for path s -> d(i), with an additional last element showing number of followed transitions

n=size(varargin{1},1); %number of nodes (all varargin{i} should have the same size)
adj=varargin{1};    %compute adj for storing adjacencies as ones and zeros (zero menas disconnected nodes)
for i=2:(nargin-2)  %for all given costs, do an "or" for finding neighboring
    adj = adj | varargin{i};
end
if nargout==3 %if desired, return adjacency matrix, but without self-loops enforced
    varargout(1)={adj};
end
for i=1:n
    adj(i,i)=1; %enforce self-loops for Dijkstra (a state is adjacenct with itself)
end
adj=double(adj);
cost_matrices=varargin; %don't add directly one array in cell varargin
cost_matrices{end+1}=adj;   %added cost (unitary adjacency)
n_c=length(cost_matrices);   %number of cost functions (it's equal to nargin-1)

%here starts Dijkstra, modified - the comparison for replacing a node is done by using a ordering relation among cost vectors (compare first, if equal compare second, and so on)
dist=Inf(n,n_c); %distances computed via all n_c costs from s to all n nodes (will be computed in while loop) - row i contains on first posiiton distance with cost1, ...
dist(s,:)=zeros(1,n_c);    %source node

visited=zeros(1,n); %if a node was considered, it has value 1 in vector "visited"
predec=zeros(1,n); %predecessor of each node

%search until all nodes are visited
while sum(visited)~=n
% %if want to search until all destination nodes are visited (not all nodes in graph): commented line from above and uncomment next one (but when set d is large there may be no gain in speed)
% while ~isempty( setdiff( d, find(visited) ))
    [~, x]=sortrows([visited' , dist]);    %ascending order of costs (ordering relation defined for cost1,cost2,...), and at the beginning are unvisited nodes (that's why visited in first column)
    % [~, x]=sortrows( dist(visited==0, :) );    %ascending order of costs (ordering relation defined for cost1,cost2,...), considering only unvisited states
    x=x(1); %consider first node (if there are more with same cost will be considered at next iterations)
    
    % x is the closest node to source (current node); we should have visited(x)==0 (otherwise, the while loop would have been finished)
    %d_m=dist(x,:);    %minimum distance vector from source to x
    visited(x)=1;
    
    neigh=find(adj(x,:)~=0 & visited==0); %unvisited neighbors of the current node x (is adj was not sparse, with "inf" instead of 0, use isfinite(adj(x,:))
    if isempty(neigh)
       continue %current node has no more unvisited neighbors, go on with next iterations (other "branches")
    end
    for i=neigh %for each unvisited neighbor, check if the distance by going through x to it is smaller than the one until now
        %use ordering relation among cost vectors
        dist_x_i=dist(x,:); %compute here distance to i by crossing through node x
%         better_x_i=0;   %a flag; assume it is not better to go through i by x
        for k=1:n_c     %take each cost, in their order
            dist_x_i(k)=dist_x_i(k)+cost_matrices{k}(x,i); %update cost k
%             if (better_x_i==0) && (dist(i,k) < dist_x_i(k))  %for cost k, it's better to go to i as until now (not through x), and until now it's not a previous cost definitely better through x
%                 break;  %break loop, don't go to i by crossing x
%             %else, it may be better to go through x (if dist(i,k) == dist_x_i(k), it may be the same, but it will be decided at next costs; if it's ">" it's definitely better through x)
%             elseif (dist(i,k) > dist_x_i(k))    %if this point was reached, previous costs were at least the same through x, by for k cost it's definitely better to go through x
%                 better_x_i=1;   %set flag; thus, none of the following costs cannot break teh loop, and all distances trgough x will be computed
%             end     %on the "==" do nothing, just continue the loop
        end
%         if better_x_i==1    %it's better to go through x, and all elements of vector dist_x_i are updated
        %all cost vector through x to i is dist_x_i; check if it's better than existing dist(i,:) in a separate function ("is_better_cost", which will be used in other procedures)
        if rmt_is_smaller_cost(dist_x_i,dist(i,:))==1   %function returns 1 if it's strictly better, 0 if its the same (equal vectors) and -1 if it's worse dist_x_i than dist(i,:)
            %*if want to replace x even if the same cost, instead of "==1" put "~=-1" on the above line - i.e., use next "if" line
        %if is_smaller_cost(dist_x_i,dist(i,:))~=-1  %replace x even if the same cost (usually multirobot paths seem better - waiting at the end, not intermediate)
            dist(i,:) = dist_x_i;   %update distance to node i (better than the previous one)
            predec(i)=x;    %x becomes the predecessor of i
        end
    end
end

%we have minimum distances from s to a set of nodes including all desired destination nodes (d)
%find the desired paths (s -> d(i)) by starting from d(i) and following predecessors
paths=cell(1,length(d));    %cell containing paths
costs=zeros(length(d),n_c); %matrix containing optimum cost vector on rows (last cost is number of transitions)
for i=1:length(d)
    if isempty( find( isinf(dist(d(i),:)) )) %#ok<EFIND> %there exists a path s -> d(i) (all costs are finite
        path=[d(i)]; %put destination node in path and start going through predecessors ("backward" search), until s is reached
        while path(1)~=s    %find all nodes between s and d(i), until s is reached (because we know that there is a path s-d(i))
            path=[predec(path(1)) path];  %add a predecesor in path, in front of the others; s will be on first position in path
        end
    else
        path=[]; %there is no path s -> d(i)
    end
    paths{i}=path;  %store path in stucture to be returned
    costs(i,:)=dist(d(i),:);    %cost vector of the i-th path
end
