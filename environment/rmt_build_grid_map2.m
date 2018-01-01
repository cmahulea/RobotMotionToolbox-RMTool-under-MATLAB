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

function [map, seq_obstacles, initial_point, final_point] = rmt_build_grid_map2(handle_axes, limits,Nobstacles)
    
fprintf('\nBuilding a grid-based map with polytopic obstacles (polygons)\n');

Nxi = limits(1);
Nx = limits(2);
Nyi = limits(3);
Ny = limits(4);

seq_obstacles = {};
map = zeros(Ny, Nx);
%imagesc(map);
%set(gca, 'Ydir', 'normal');
%grid;
%binary_cmap = [1 1 1; 1 0 0];
%colormap(binary_cmap);
%caxis([0 1]);
%figure(gcf)
%set(gcf, 'Name', 'build_grid_map');

env_bounds=[Nxi,Nx,Nyi,Ny];

%uiwait(msgbox(sprintf('\nFor defining a region:\n\t - left-click = pick a vertex\n\t - right-click = pick last vertex\n\nRegions should be convex and non-overlapping\n'),...
%    'Robot Motion Toolbox','modal'));

axes(handle_axes);
axis(env_bounds);
hold on
grid on

uiwait(msgbox(sprintf('\nFor defining a region:\n\t - left-click = pick a vertex\n\t - right-click = pick last vertex\n\nRegions must be squared/rectangular, convex and non-overlapping\n'),...
    'Robot Motion Toolbox','modal'));

for(i=1:Nobstacles)
    %read obstacle's vertices
    j = 1; %j = no. of vertexes for current object
    but = 1;
    while but==1
        [x,y,but]=ginput(1);
        x = round(x);
        y = round(y);
        plot(x,y,'.k')
        objects{i}(:,j)=[x;y];
        j=j+1;
    end    
    %creating convex obstacles & drawing them
    k=convhull(objects{i}(1,:),objects{i}(2,:));
    objects{i}=objects{i}(:,k(1:length(k)-1));
    pause(0.3)
    fill(objects{i}(1,:),objects{i}(2,:),'k','FaceAlpha',0.5); %or functia patch (similara cu fill)
    %drawnow
    %fprintf('\nclick a sequence of points, <enter> when done\n');
    %title('click a sequence of points, <enter> when done');
    %xy = ginput;            
    %xy = round(xy);         
    [X,Y] = meshgrid(1:Nx, 1:Ny);
    X = X;
    Y = Y;
    map = map + inpolygon(X, Y, x(:), y(:));        
    %seq_obstacles(i) = {[x y]};
    % we check that the maximum value of the map is 1 (obstacles)
    map = min(map, 1);
    % update the map figure
    %set(handle_axes, 'CData', map);
    %aux = [x y];
    %for(j=1:3)
    %  aa = [aux(j,1),aux(j+1,1)];
    %  bb = [aux(j,2),aux(j+1,2)];
    %  plot(aa,bb,'b','LineWidth',4);        
   %end
   %aa = [aux(1,1),aux(4,1)];
   %bb = [aux(1,2),aux(4,2)];
   %plot(aa,bb,'b','LineWidth',4);           
end%for

seq_obstacles = {};
for(k=1:Nobstacles)
    a = objects{k};
    seq_obstacles(k) = {a'};
end

uiwait(msgbox(sprintf('\nChoose the initial and goal points with right click.\n'),'Robot Motion Toolbox','modal'));

for i=1:2   
    j=1; %j = no. of vertexes for current object
    but=1;
    %while but==1
    %    [x,y,but]=ginput(1);
    %    x = round(x);       
    %    y = round(y);       
    %end      
    point_ok = 0;
    while(point_ok == 0)
        but=1;
        while but==1
            [x,y,but]=ginput(1);
            x = round(x);
            y = round(y);
        end
        in = 0;
        for(ii=1:Nobstacles)
            in = in + inpolygon(x,y,objects{ii}(1,:),objects{ii}(2,:));
        end 
        if(in>0)
            uiwait(msgbox(sprintf('\nInvalid point!\n'),'Robot Motion Toolbox','modal'));
        else
            point_ok = 1;
        end
    end        
    if i == 1
        plot(x,y,'or','LineWidth',3);
        initial_point = [x,y];
    else
        plot(x,y,'xk','LineWidth',4);
        final_point = [x,y];
    end
    %creating convex obstacles & drawing them
%     k=convhull(objects{i}(1,:),objects{i}(2,:));
%     objects{i}=objects{i}(:,k(1:length(k)-1));
%     pause(0.3)
%     fill(objects{i}(1,:),objects{i}(2,:),'k','FaceAlpha',0.5); %or functia patch (similara cu fill)
end

disp('The environment has been built (V-graph).');

end
