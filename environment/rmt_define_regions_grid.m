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

function [objects,varargout] = rmt_define_regions_grid(env_bounds,reg_no,handle_axes,robot_no,define_final,obs_size)
% This function lets the user to define regions via mouse clicks
% input arguments (2 or 5): env_bounds,reg_no,[handle_axes,robot_no,define_final]
% output arguments (1 or 3): objects,[initial_point,final_point]
% env_bounds=[x_min,x_max,y_min,y_max];

for i = env_bounds(1): obs_size : env_bounds(2)
    plot([i i],[env_bounds(3) env_bounds(4)],':r');
end

for i = env_bounds(3): obs_size : env_bounds(4)
    plot([env_bounds(1) env_bounds(2)],[i i],':r');
end

uiwait(msgbox(sprintf('\nUse left or right click to select a region'),...
    'Robot Motion Toolbox','modal'));

try
    axes(handle_axes);
catch
    figure();
end
axis(env_bounds);
hold on
grid on

%read obstacle's regions
for i=1:reg_no   %i = object number
    j=1; %j = no. of vertexes for current object
    added = 0;
    while (added == 0)
        [x,y,~]=ginput(1);
        if (x >= env_bounds(1) && x <= env_bounds(2) && y >= env_bounds(3) && y <= env_bounds(4)) %inside bounds
            objects{i}=obs_size*[floor(x/obs_size) floor(x/obs_size)+1 floor(x/obs_size)+1 floor(x/obs_size);
                floor(y/obs_size) floor(y/obs_size) floor(y/obs_size)+1 floor(y/obs_size)+1];
            added = 1;
            for k = 1 : i-1
                if (norm(mean(objects{i},2)-mean(objects{k},2))<10^5*eps) %region already defined
                    added = 0;
                    uiwait(msgbox(sprintf('\nRegions of interest should be disjoint'),...
                        'Robot Motion Toolbox','modal'));
                    break;
                end
            end
        end
    end
    %drawing the regions
    pause(0.3)
    fill(objects{i}(1,:),objects{i}(2,:),'b-','FaceAlpha',0.5); %or functia patch (similara cu fill)
end

if (define_final==1)
    uiwait(msgbox(sprintf('\nChoose the initial and goal points with right click.\n'),'Robot Motion Toolbox','modal'));
else
    uiwait(msgbox(sprintf('\nChoose the initial points of the robots with right click.\n'),'Robot Motion Toolbox','modal'));
end
    
initial_point={};
final_point = {};
    
for k = 1:robot_no
    if (define_final==1)
        for i=1:2
            point_ok = 0;
            while(point_ok == 0)
                but=1;
                while but==1
                    [x,y,but]=ginput(1);
                end
                point_ok = 1;
                in = obs_size*[floor(x/obs_size) floor(x/obs_size)+1 floor(x/obs_size)+1 floor(x/obs_size);
                    floor(y/obs_size) floor(y/obs_size) floor(y/obs_size)+1 floor(y/obs_size)+1];
                for ll = 1 : length(objects)
                    if (norm(mean(objects{ll},2)-mean(in,2))<10^5*eps) %point inside a region of interest
                        uiwait(msgbox(sprintf('\nInitial point of the robots cannot be in a region of interest'),...
                            'Robot Motion Toolbox','modal'));
                        point_ok = 0;
                        break;
                    end
                end
                for ll = 1 : length(initial_point)
                    if (norm(initial_point{ll} - mean(in,2)) <= 10^5*eps)
                        uiwait(msgbox(sprintf('\nMaximum one robot inside a region'),...
                            'Robot Motion Toolbox','modal'));
                        point_ok = 0;
                        break;
                    end
                end
                for ll = 1 : length(final_point)
                    if (norm(final_point{ll} - mean(in,2)) <= 10^5*eps)
                        uiwait(msgbox(sprintf('\nMaximum one robot inside a region'),...
                            'Robot Motion Toolbox','modal'));
                        point_ok = 0;
                        break;
                    end
                end
            end
            if (i == 1)
                initial_point{k} = mean(in,2);
                plot(initial_point{k}(1),initial_point{k}(2),'or','LineWidth',3);
            else
                final_point{k} = mean(in,2);
                plot(final_point{k}(1),final_point{k}(2),'xk','LineWidth',3);
            end
        end
    else
        point_ok = 0;
        while(point_ok == 0)
            but=1;
            while but==1
                [x,y,but]=ginput(1);
            end
            point_ok = 1;
            in = obs_size*[floor(x/obs_size) floor(x/obs_size)+1 floor(x/obs_size)+1 floor(x/obs_size);
                floor(y/obs_size) floor(y/obs_size) floor(y/obs_size)+1 floor(y/obs_size)+1];
            for ll = 1 : length(objects)
                if (norm(mean(objects{ll},2)-mean(in,2))<10^5*eps) %point inside a region of interest
                    uiwait(msgbox(sprintf('\nInitial point of the robots cannot be in a region of interest'),...
                        'Robot Motion Toolbox','modal'));
                    point_ok = 0;
                    break;
                end
            end
            for ll = 1 : length(initial_point)
                if (norm(initial_point{ll} - mean(in,2)) <= 10^5*eps)
                    uiwait(msgbox(sprintf('\nMaximum one robot inside a region'),...
                        'Robot Motion Toolbox','modal'));
                    point_ok = 0;
                    break;
                end
            end
            for ll = 1 : length(final_point)
                if (norm(final_point{ll} - mean(in,2)) <= 10^5*eps)
                    uiwait(msgbox(sprintf('\nMaximum one robot inside a region'),...
                        'Robot Motion Toolbox','modal'));
                    point_ok = 0;
                    break;
                end
            end
        end
        initial_point{k} = mean(in,2);
        plot(initial_point{k}(1),initial_point{k}(2),'or','LineWidth',3);
    end
end
%    final_point{k}=mean(obs_size*[pos_x pos_x+1 pos_x+1 pos_x; pos_y pos_y pos_y+1 pos_y+1],2);

varargout(1)={initial_point};
varargout(2)={final_point};
end

