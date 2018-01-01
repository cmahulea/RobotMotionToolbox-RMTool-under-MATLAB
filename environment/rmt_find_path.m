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


function [path, cost] = find_path(adj,s,d)
%find the path with minimum cost (shortest path) in a graph, from a source node s to a node (d), and the corresponding cost
%the graph is known through an adjacency matrix, where adj(i,j)=cost for going from i to j; exception if adj(i,j)==0 - see below
%Dijkstra's algorithm is used (since it finds minimal costs for all nodes in graph), with the following small modification:
%if adj(i,j)==0, (i~=j), then there is no transition from i to j, so this 0 is not a cost
%(this is done because we want that adj be a sparse matrix, for managing more states;
%we will check to not have costs less than precision, which could be approximated with 0)
%s - index for soure node (scalar), d - index for destination node
%path will be a vector giving the path s -> d
%cost gives the cost for path s -> d(i)

n=size(adj,1); %number of nodes

visited=zeros(1,n); %if a node was considered, it has value 1 in vector "visited"
dist=Inf(1,n); %distances from s to nodes (will be modified)
dist(s)=0;
predec=zeros(1,n); %predecessor of each node

%search until all nodes are visited
while sum(visited)~=n
    d_m=min(dist(find(visited==0))); %minimum distance from source, considerrind only unvisited states
    x=find(dist==d_m & visited==0); %x is the closest node to source
    x=x(1); %if there are more nodes at the same distance (they will be considered at following iterations)
    visited(x)=1;
    neigh=find(adj(x,:)~=0 & visited==0); %unvisited neighbors of the current node x (is adj was not sparse, with "inf" instead of 0, use isfinite(adj(x,:))
    if length(neigh)==0
       continue %current node has no more unvisited neighbors, go on with next iterations (other "branches")
    end
    for i=neigh %for each unvisited neighbor, check if the distance by going through x to it is smaller than the one until now
        if (dist(i) > dist(x)+adj(x,i))
            dist(i)=dist(x)+adj(x,i);   %update distance to node i (smaller than the previous one)
            predec(i)=x;    %x becomes the predecessor of i
        end
    end
end

%we have minimum distances from s to all nodes in graph
%find the desired path (s -> d) by starting from d and following predecessors
if dist(d)~=Inf %there exists a path s -> d
    path=[d]; %put destination node in path and start going through predecessors ("backward" search), until s is reached
    while path(1)~=s    %find all nodes between s and d(i), until s is reached (because we know that there is a path s->d)
        path=[predec(path(1)) path];  %add a predecesor in path, in front of the others; s will be on first position in path
    end
else
    path=[]; %there is no path s -> d
end
cost=dist(d);    %cost of the path
