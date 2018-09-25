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
%   Last modification February 14, 2018.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function [objects,varargout] = rmt_define_regions(env_bounds,reg_no,varargin)
% This function lets the user to define regions via mouse clicks
% input arguments (2 or 5): env_bounds,reg_no,[handle_axes,robot_no,define_final]
% output arguments (1 or 3): objects,[initial_point,final_point]
% env_bounds=[x_min,x_max,y_min,y_max];

switch nargin
    case 2
        robot_no = 0;
        if nargout ~=1
            fprintf('\nIncorrect number of output arguments for function "rmt_define_regions" (should be 1)!\n');
            return;
        end
    case 5
        handle_axes=varargin{1};
        robot_no = varargin{2};
        define_final = varargin{3};
        if nargout ~=3
            fprintf('\nIncorrect number of output arguments for function "rmt_define_regions" (should be 3)!\n');
            return;
        end
    otherwise
        fprintf('\nIncorrect number of input arguments for function "rmt_define_regions" (should be 2 or 5)!\n');
        return;
end

if (robot_no == 1)
    uiwait(msgbox(sprintf('\nFor defining a region:\n\t - left-click = pick a vertex\n\t - right-click = pick last vertex\n\nRegions should be convex and non-overlapping\n'),...
        'Robot Motion Toolbox','modal'));
else
    uiwait(msgbox(sprintf('\nFor defining a region:\n\t - left-click = pick a vertex\n\t - right-click = pick last vertex'),...
        'Robot Motion Toolbox','modal'));
end

try
    axes(handle_axes);
catch
    figure();
end
axis(env_bounds);
hold on
grid on

%read obstacle's vertices
for i=1:reg_no   %i = object number
    j=1; %j = no. of vertexes for current object
    but=1;
    while (but==1 || j<4)
        [x,y,but]=ginput(1);
        if (x >= env_bounds(1) && x <= env_bounds(2) && y >= env_bounds(3) && y <= env_bounds(4)) %inside bounds
            plot(x,y,'.k')
            objects{i}(:,j)=[x;y];
            j=j+1;
        else
            but=1; %points outside environment bounds are ignored; do not store "but" for wrong points
        end
    end
    
    %creating convex obstacles & drawing them
    k=convhull(objects{i}(1,:),objects{i}(2,:));
    objects{i}=objects{i}(:,k(1:length(k)-1));
    pause(0.3)
    fill(objects{i}(1,:),objects{i}(2,:),'b-','FaceAlpha',0.5); %or functia patch (similara cu fill)
end

if nargin == 5
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
%                     in = 0;
                    in = 1 - inpolygon(x,y,[env_bounds(1),env_bounds(2),env_bounds(2),env_bounds(1)],[env_bounds(3),env_bounds(3),env_bounds(4),env_bounds(4)]);    %in=1 if point is outside bounds
                    for(ii=1:reg_no)
                        in = in + inpolygon(x,y,objects{ii}(1,:),objects{ii}(2,:));
                    end
                    if(in>0)
                        uiwait(msgbox(sprintf('\nInvalid point!\n'),'Robot Motion Toolbox','modal'));
                    else
                        point_ok = 1;
                    end
                end
                %plot(x,y,'ob')
                if i == 1
                    plot(x,y,'or','LineWidth',3);
                    initial_point{k} = [x,y];
                else
                    plot(x,y,'xk','LineWidth',3);
                    final_point{k} = [x,y];
                end
            end
        else
            point_ok = 0;
            while(point_ok == 0)
                but=1;
                while but==1
                    [x,y,but]=ginput(1);
                end
%                 in = 0;
                in = 1 - inpolygon(x,y,[env_bounds(1),env_bounds(2),env_bounds(2),env_bounds(1)],[env_bounds(3),env_bounds(3),env_bounds(4),env_bounds(4)]);    %in=1 if point is outside bounds
                for(ii=1:reg_no)
                    in = in + inpolygon(x,y,objects{ii}(1,:),objects{ii}(2,:));
                end
                if(in>0)
                    uiwait(msgbox(sprintf('\nInvalid point!\n'),'Robot Motion Toolbox','modal'));
                else
                    point_ok = 1;
                end
            end
            plot(x,y,'or','LineWidth',3);
            initial_point{k} = [x,y];
        end
    end
    
    varargout(1)={initial_point};
    varargout(2)={final_point};
end
end

