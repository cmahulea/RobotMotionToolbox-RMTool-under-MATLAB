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
%   First version released on May, 2021.
%   Last modification May 07, 2021.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function [status] = rmt_add_robot

status=0;
IsReach = get(findobj(gcf,'Tag','reach'),'Value');
if (IsReach == 1)
    uiwait(errordlg('This option is only for multi-robot systems!',...
        'Robot Motion Toolbox','modal'));
    return;
end
uiwait(msgbox(sprintf('\nChoose the initial point of the new robot with right click.\n'),'Robot Motion Toolbox','modal'));
data = get(gcf,'UserData');
point_ok = 0;
while(point_ok == 0)
    but=1;
    while but==1
        [x,y,but]=ginput(1);
    end
    in = 0;
    for ii=1:data.Nobstacles
        in = in + inpolygon(x,y,data.obstacles{ii}(1,:),data.obstacles{ii}(2,:));
    end
    if(in>0)
        uiwait(msgbox(sprintf('\nInvalid point!\n'),'Robot Motion Toolbox','modal'));
    else
        point_ok = 1;
    end
end
plot(x,y,'or','LineWidth',3);
data.initial{length(data.initial)+1} = [x,y];
for j=1:length(data.T.Vert)   %indices of starting cell
    if inpolygon(x,y,data.T.Vert{j}(1,:),data.T.Vert{j}(2,:))
        data.RO = [data.RO j];
        data.T.RO = [data.T.RO j];
        break;
    end
end
data.T.m0(j) = data.T.m0(j) + 1;  %number of robots initially in state i
data.orientation=[data.orientation 0];
set(gcf,'UserData',data);

%update the plots
cla(data.handle_env);
rmt_plot_environment(data.obstacles,data.frame_limits,data.T.Vert);
rmt_represent_atomic_props(data.T.Vert,data.propositions);    %represent all atomic props
for r=1:length(data.RO)
    plot(data.initial{r}(1),data.initial{r}(2),'Color',data.rob_plot.line_color{r},...
        'LineStyle',data.rob_plot.line{r},...
        'LineWidth',data.rob_plot.line_width{r},...
        'Marker',data.rob_plot.marker{r},...
        'MarkerFaceColor',data.rob_plot.face_color{r});
end
status = 1;
return