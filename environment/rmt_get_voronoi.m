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


function [traj] = rmt_get_voronoi(handle_axes, limits, Num_Object, Start, ...
    Goal, X_Total_points, Y_Total_points, All_cells_Number, Cell_start, X1)

%clear all;
%close all;
%clc;

Nxi = limits(1);
Nx = limits(2);
Nyi = limits(3);
Ny = limits(4);
env_bounds=[Nxi,Nx,Nyi,Ny];

axes(handle_axes);
axis(env_bounds);
hold on
grid on


%specify file name here to load from
%LOAD_FILE_NAME = 'Obstacle_config';

%load(strcat(strcat(LOAD_FILE_NAME)));



%Code for drawing obstale configuration
%for i=1:Num_Object
%    for r=1:length(X1{i})
%       a=r;
%       if(r==length(X1{i}))
%           b=1;
%       else
%           b=r+1;
%       end
%       x=[X1{i}(a,1) X1{i}(b,1)];
%       y=[X1{i}(a,2) X1{i}(b,2)];
%       plot(x,y,'Color', 'Black');
%       hold on;
%    end
%end

%Code for taking Start and End point as input
%Start = ginput(1);
%plot(Start(1),Start(2),'--go','MarkerSize',10,'MarkerFaceColor','g');
%drawnow;
%Goal  = ginput(1);
%plot(Goal(1),Goal(2),'--ro','MarkerSize',10,'MarkerFaceColor','r');
%drawnow;

%Uncomment following to Draw voronoi diagram of point obstacles
%voronoi(X_Total_points,Y_Total_points);
%Getting Parameters of Voronoi Diagram
[Voro_Vertex,Voro_Cell] = voronoin([X_Total_points' Y_Total_points']);

k=1;
temp=0;
for i=1:length(All_cells_Number)
    L=length(Voro_Cell{i});
  for j=1:L
      a=Voro_Cell{i}(j);
      if(j==L)
          b=Voro_Cell{i}(1);
      else
          b=Voro_Cell{i}(j+1);
      end
      for l=1:Num_Object
          if(temp==1)
              temp=0;
              break;
          end
          if (l==All_cells_Number(i));
              continue;
          end
          for m=Cell_start(l):Cell_start(l+1)-2%-2
              if((~isempty(find(Voro_Cell{m}==a)))&(~isempty(find(Voro_Cell{m}==b))))
                  Temp_Edge(k,:)=[a b];
                  k=k+1;
                  temp=1;
                  break;
              end
          end     
      end
  end    
end

Temp_Edge=unique(Temp_Edge,'rows');

%figure;
%axis([0 100 0 100]);
%hold on;

for i=1:length(Temp_Edge)
    Edge_X1(i)=Voro_Vertex(Temp_Edge(i,1),1);
    Edge_X2(i)=Voro_Vertex(Temp_Edge(i,2),1);
    Edge_Y1(i)=Voro_Vertex(Temp_Edge(i,1),2);
    Edge_Y2(i)=Voro_Vertex(Temp_Edge(i,2),2);
    plot([Edge_X1(i) Edge_X2(i)],[Edge_Y1(i) Edge_Y2(i)],'color',[.8 .8 .8]);
end

%% =================================================================================
% FINAL_PATH.M
% =================================================================================

%Minimum Distance

Vertex = unique(Temp_Edge);
N = length(Vertex);
M = length(Temp_Edge);

for i=1:N
    Vertex_Cord(i,:)=Voro_Vertex(Vertex(i),:);
    Start_distance(i)=norm(Start-Vertex_Cord(i,:));
    Goal_distance(i)=norm(Goal-Vertex_Cord(i,:));
end


Voro_Graph = inf*ones(N);

%figure;
%axis([0 100 0 100]);
%hold on;

for i = 1:M
    a= find(Vertex==Temp_Edge(i,1));
    b= find(Vertex==Temp_Edge(i,2));
    Distance = norm(Vertex_Cord(a,:)-Vertex_Cord(b,:));
    Voro_Graph(a,b)=Distance;
    Voro_Graph(b,a)=Distance;
%     Voro_Graph(a,b)=1;
%     Voro_Graph(b,a)=1;
    
    x=[Vertex_Cord(a,1) Vertex_Cord(b,1)];
    y=[Vertex_Cord(a,2) Vertex_Cord(b,2)];
    
    %plot(x,y,'color','Green','LineWidth',2);
end

for i=1:N
    Start_distance(i)=norm(Start-Vertex_Cord(i,:));
    Goal_distance(i)=norm(Goal-Vertex_Cord(i,:));
end

[Dummy Index_Start]=min(Start_distance);
[Dummy Index_Goal]=min(Goal_distance);
path = dijkstra(Voro_Graph,Index_Start,Index_Goal);

for i=1:Num_Object
    for r=1:length(X1{i})
       a=r;
       if(r==length(X1{i}))
           b=1;
       else
           b=r+1;
       end
       x=[X1{i}(a,1) X1{i}(b,1)];
       y=[X1{i}(a,2) X1{i}(b,2)];
       plot(x,y,'g');
       hold on;
    end
end
drawnow;

plot(Start(1),Start(2),'pw','Markersize',13, 'Color', 'g');
plot(Goal(1),Goal(2), 'pw','Markersize',13, 'Color', 'b');
 
 %figure(1);
 %axis([0 100 0 100]);
 hold on;
 
 for i=1:length(Temp_Edge)
    Edge_X1(i)=Voro_Vertex(Temp_Edge(i,1),1);
    Edge_X2(i)=Voro_Vertex(Temp_Edge(i,2),1);
    Edge_Y1(i)=Voro_Vertex(Temp_Edge(i,1),2);
    Edge_Y2(i)=Voro_Vertex(Temp_Edge(i,2),2);
    plot([Edge_X1(i) Edge_X2(i)],[Edge_Y1(i) Edge_Y2(i)],'color',[.7 .7 .7],'LineWidth',2);
end
 
 x=[Start(1) Vertex_Cord(path(1),1)];
 y=[Start(2) Vertex_Cord(path(1),2)];
 plot(x,y,'-','color','r','LineWidth',2);
 drawnow;
 
 traj = [];
 traj = [traj;[x', y']];
 %aux_x = traj(end-1,1);
 %aux_y = traj(end-1,2);
 %traj(end-1,1) = traj(end-2,1);
 %traj(end-1,2) = traj(end-2,2);
 %traj(end-2,1) = aux_x;
 %traj(end-2,2) = aux_y;
 
 
 for i=1:length(path)-2
 x=[Vertex_Cord(path(i),1) Vertex_Cord(path(i+1),1)];
 y=[Vertex_Cord(path(i),2) Vertex_Cord(path(i+1),2)];
 plot(x,y,'-','color','r','LineWidth',2);
 %drawnow;
 hold on;
 traj = [traj;[x', y']];
 end
 
 x=[Vertex_Cord(path(i+1),1) Goal(1)];
 y=[Vertex_Cord(path(i+1),2) Goal(2)];
 plot(x,y,'-','color','r','LineWidth',2);
 drawnow;
 
 traj = [traj;[x', y']];
 

end%function

%% Dijkstra function
function [shortestPath, totalCost] = dijkstra(netCostMatrix, s, d)
%==============================================================
% shortestPath: the list of nodes in the shortestPath from source to destination;
% totalCost: the total cost of the  shortestPath;
% farthestNode: the farthest node to reach for each node after performing the routing;
% n: the number of nodes in the network;
% s: source node index;
% d: destination node index;
%==============================================================
%  Code by:
% ++by Xiaodong Wang
% ++23 Jul 2004 (Updated 29 Jul 2004)
% ++http://www.mathworks.com/matlabcentral/fileexchange/5550-dijkstra-shortest-path-routing
% Modifications (simplifications) by Meral Shirazipour 9 Dec 2009
%==============================================================
n = size(netCostMatrix,1);
for i = 1:n
    % initialize the farthest node to be itself;
    farthestPrevHop(i) = i; % used to compute the RTS/CTS range;
    farthestNextHop(i) = i;
end

% all the nodes are un-visited;
visited(1:n) = false;

distance(1:n) = inf;    % it stores the shortest distance between each node and the source node;
parent(1:n) = 0;

distance(s) = 0;
for i = 1:(n-1),
    temp = [];
    for h = 1:n,
         if ~visited(h)  % in the tree;
             temp=[temp distance(h)];
         else
             temp=[temp inf];
         end
     end;
     [t, u] = min(temp);      % it starts from node with the shortest distance to the source;
     visited(u) = true;         % mark it as visited;
     for v = 1:n,                % for each neighbors of node u;
         if ( ( netCostMatrix(u, v) + distance(u)) < distance(v) )
             distance(v) = distance(u) + netCostMatrix(u, v);   % update the shortest distance when a shorter shortestPath is found;
             parent(v) = u;     % update its parent;
         end;             
     end;
end;

shortestPath = [];
if parent(d) ~= 0   % if there is a shortestPath!
    t = d;
    shortestPath = [d];
    while t ~= s
        p = parent(t);
        shortestPath = [p shortestPath];
        
        if netCostMatrix(t, farthestPrevHop(t)) < netCostMatrix(t, p)
            farthestPrevHop(t) = p;
        end;
        if netCostMatrix(p, farthestNextHop(p)) < netCostMatrix(p, t)
            farthestNextHop(p) = t;
        end;

        t = p;      
    end;
end;

totalCost = distance(d);
%return;

end %dijkstra
