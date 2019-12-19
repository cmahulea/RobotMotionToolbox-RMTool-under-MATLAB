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
%   Last modification February 19, 2018.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

% Function to create Random Enviroment
function [objects,initial_point,final_point] = rmt_random_env_grid(handle_axes,obs_no,obs_size,dest_no,rob_no,limits,colors)
axes(handle_axes);
axis(limits);
hold on
grid on

objects={};
objects_bound={};

initial_point={};
initial_bound={};

final_point={};
final_bound={};

%% Obstacles

x = (limits(2)-limits(1)) / obs_size;
y = (limits(4)-limits(3)) / obs_size;

random = unique(randi(x*y, 1, obs_no))-1;
while (length(random) < obs_no+max(rob_no,2))
    random = unique([random randi(x*y, 1, obs_no+max(rob_no,2)-length(random))-1]);
end

for i=1:obs_no
    cell = random(i);
    pos_x = mod(cell,x);
    pos_y = floor(cell/x);
    objects{i}=obs_size*[pos_x pos_x+1 pos_x+1 pos_x; pos_y pos_y pos_y+1 pos_y+1];
  
    %draw the region
    pause(0.001);
    fill(objects{i}(1,:),objects{i}(2,:),'b-','FaceAlpha',0.5); %or functia patch (similara cu fill)
end


%% Initial and final points

for k=1:rob_no %Initial points
    i = i + 1; 
    cell = random(i);
    pos_x = mod(cell,x);
    pos_y = floor(cell/x);
    initial_point{k}=mean(obs_size*[pos_x pos_x+1 pos_x+1 pos_x; pos_y pos_y pos_y+1 pos_y+1],2);
    if (rob_no > 1)
        final_point{k} = initial_point{k};
    end
end

if (rob_no == 1)
    i = i + 1;
    cell = random(i);
    pos_x = mod(cell,x);
    pos_y = floor(cell/x);
    final_point{k}=mean(obs_size*[pos_x pos_x+1 pos_x+1 pos_x; pos_y pos_y pos_y+1 pos_y+1],2);
end
   

%% Plotting points

for i=1:length(initial_point)   
    plot(initial_point{i}(1),initial_point{i}(2),'color',colors{i},'marker','o','Linewidth',2);
    text((initial_point{i}(1)+0.2),(initial_point{i}(2)+0.2),{num2str(i)});
    hold on;
end

if (rob_no == 1)
    for i=1:length(final_point)
        plot(final_point{i}(1),final_point{i}(2),'color','k','marker','x','Linewidth',2);
        text((final_point{i}(1)+0.2),(final_point{i}(2)+0.2),{char(64+i)});
        hold on;
    end
end
pause(0.1)

end




