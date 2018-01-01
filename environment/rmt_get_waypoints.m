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


function traj = rmt_get_waypoints(handle_ax, env_bounds,varargin,traj_ini)

%hold on
%set(gca,'Box','on');

uiwait(msgbox(sprintf('\nFirst: Define a piecewise linear trajectory (at least 2 points are required): \n\t Left-click = pick an intermediary point. \n\t Right-click = pick last point.\n\t The first point is the origin and the last point the goal.\n'),...
    'Robot Motion Toolbox','modal'));

axes(handle_ax);
axis(env_bounds);
hold on
grid on

% init_wrld_h=figure();
%axis(world_dim);
%plot(traj_ini(1),traj_ini(2),'ob','LineWidth',3);
%legend('Robot starting position');
%title('Define a piecewise linear trajectory: Left-click = intermediary point Right-click = last point');


but=1;
%traj=[traj_ini(1);traj_ini(2)];
traj = [];
while but==1
    [x,y,but]=ginput(1);
    x=round(x*2)/2;
    y=round(y*2)/2;
    plot(x,y,'.k')
    traj=[traj , [x;y] ];
end

if length(varargin)>0   %distance between two successive points specificed; create intermediate points on trajectory
    min_dist=varargin;
    new_traj=[traj(1,1);traj(2,1)];
    new_traj=[];
    for i=1:(size(traj,2)-1)
        nr_points=ceil(norm(traj(:,i+1)-traj(:,i))/min_dist)+1; %number of intermetiate points
        nr_points=max(nr_points,2); %at least 2 points
        interm=linspace(0,1,nr_points); %equidistant values between 0 and 1
        %construct and add intermediate points
        new_traj=[new_traj, [(1-interm)*traj(1,i) + interm*traj(1,i+1) ; (1-interm)*traj(2,i) + interm*traj(2,i+1)] ];
    end
    traj=new_traj;
    plot(traj(1,:),traj(2,:),'r');
else %traj remains only in given points, plot dashed
    plot(traj(1,:),traj(2,:),'--r');
end
end%function
