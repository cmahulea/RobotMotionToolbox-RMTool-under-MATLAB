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

function rmt_generate_partitions(objects,initial_points)
% let the user to introduce the regions of interest or generate random

data = get(gcf,'UserData');
limits = data.frame_limits;
robot_no = length(initial_points);

[T,propositions] = rmt_tr_sys_partition(objects,limits(2),limits(4));  %create partition and represent it
rmt_plot_environment(objects,data.frame_limits,T.Vert);
rmt_represent_atomic_props(T.Vert,propositions);    %represent all atomic props
rob_plot = data.rob_plot;

%Creation of cell's matrix with (n,m) position
%The firtst row have the number of robot that the user enters
current_pos=cell(1,robot_no);
for r=1:robot_no
    current_pos{r}=initial_points{r};%R_trajs{r}(:,1); %current_pos - initial (current)
end
for r=1:robot_no
    plot(current_pos{r}(1),current_pos{r}(2),'Color',rob_plot.line_color{r},'LineStyle',rob_plot.line{r},'LineWidth',rob_plot.line_width{r},'Marker',rob_plot.marker{r},'MarkerFaceColor',rob_plot.face_color{r});
end
uiwait(msgbox('Partition was constructed based on defined regions of interest.','Robot Motion Toolbox','modal'));
%Each triangular region has three vertices, shown in structure C according to this order:
%1. Central;
%2. Right;
%3. Left;
%"C" is an array of 1 x number of regions. Each column has two elements (X and Y coordinates for vertices)
C = T.Vert;
RO=[];
for i = 1 : robot_no
    temp = initial_points{i};
    for j=1:length(C)   %indices of starting & final cells
        if inpolygon(temp(1),temp(2),C{j}(1,:),C{j}(2,:))
            RO(i)=j;
            break;
        end
    end
end
%T.RO contains the information about the position of initial marking (ex. the initial marking is present
%in region P5 e P11),
%instead T.M0 contains the  initial marking, in which region
%the inserted robot is present.
T.m0=zeros(length(T.Q),1);    %initial marking (for PN)
T.RO = RO;
for i=T.Q
    T.m0(i)=sum(ismember(RO,i));  %number of robots initially in state i
end
Tr = rmt_quotient_T(T); % quotient of partition T, with fewer states
%(based on collapsing states with same observables in same connected component
% with same obs)
orient = [];
for i = 1 : length(initial_points)
    orient(i) = 0;
end
data.Nobstacles = length(objects);
data.obstacles = objects;
data.T=T;
data.Tr=Tr;
data.propositions=propositions;
data.RO = RO;
data.initial=initial_points;
data.final=initial_points;
data.orientation=orient;
set(gcf,'UserData',data);
return;