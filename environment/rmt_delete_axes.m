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
%   First version released on October, 2018.
%   Last modification October 28, 2018.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function rmt_delete_axes(data,mission)
%Delete all axes and invalidate the ones related to motion control -> version April 2018
%if mission=0 -> reachability mission
%if mission=1 -> ltl mission
%if mission=2 -> boolean mission

set(findobj(gcf,'Tag','reach'),'Value',0);
set(findobj(gcf,'Tag','ltl'),'Value',0);
set(findobj(gcf,'Tag','boolean'),'Value',0);
if (mission == 0)
    set(findobj(gcf,'Tag','reach'),'Value',1);
    set(findobj(gcf,'Tag','pathpoints'),'Enable','on');
    set(findobj(gcf,'Tag','triang'),'Enable','on');
    set(findobj(gcf,'Tag','rect'),'Enable','on');
    set(findobj(gcf,'Tag','poly'),'Enable','on');
    set(findobj(gcf,'Tag','trapez'),'Enable','on');
    set(findobj(gcf,'Tag','waypoints'),'Enable','on');
elseif (mission == 1)
    set(findobj(gcf,'Tag','ltl'),'Value',1);
elseif (mission == 2)
    set(findobj(gcf,'Tag','boolean'),'Value',1);
else
    uiwait(errordlg(sprintf('\nWrong input parameter when trying to delete axes objects'),'Robot Motion Toolbox','modal'));
    error('\nWrong input parameter when trying to delete axes objects\n');
end

if ((mission == 1) || (mission == 2))
    set(findobj(gcf,'Tag','pathpoints'),'Value',1,'Enable','off');
    set(findobj(gcf,'Tag','triang'),'Enable','off','Value',1);
    set(findobj(gcf,'Tag','rect'),'Enable','off','Value',0);
    set(findobj(gcf,'Tag','poly'),'Enable','off','Value',0);
    set(findobj(gcf,'Tag','trapez'),'Enable','off','Value',0);
    set(findobj(gcf,'Tag','waypoints'),'Enable','off','Value',1);
end

limits = data.frame_limits;
%we clean the workspace figure
cla(data.handle_env);
set(data.handle_env,'xlim',[limits(1) limits(2)],'ylim',[limits(3) limits(4)],'XGrid','on','YGrid','on');
%we clean the orientation figure
cla(data.handle_ori);
set(data.handle_ori,'XGrid','on','YGrid','on','Visible','off');
%we clean the velocities figure
cla(data.handle_vel);
set(data.handle_vel,'XGrid','on','YGrid','on','Visible','off');
%we clean the steering angle figure
cla(data.handle_ang);
set(data.handle_ang,'XGrid','on','YGrid','on','Visible','off');
%enable the text information box
set(data.handle_text,'Visible','on','String','');
for i = 1 : 6
    set(data.handles_control(i),'Visible','off');
end
return