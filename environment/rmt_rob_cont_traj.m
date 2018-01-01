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

function rob_traj = rmt_rob_cont_traj(T,Runs,x0)

rob_traj=cell(1,length(Runs));
for r=1:length(Runs)    %for each robot
    rob_traj{r}=zeros(2,length(Runs{r})+1);
    
    rob_traj{r}(:,1) = x0{r};
    
    for i=2:length(Runs{r})
        if Runs{r}(i-1)~=Runs{r}(i)   %this should be always true
            rob_traj{r}(:,i) = [T.mid_X(Runs{r}(i-1),Runs{r}(i)) ; T.mid_Y(Runs{r}(i-1),Runs{r}(i))];  %middle point of segment shared by two successive states
        end
    end
    
    if length(Runs{r})>1    %robot r has moved
        rob_traj{r}(:,end) = mean(T.Vert{Runs{r}(end)},2);   %centroid of last state
    else
        rob_traj{r}(:,end) = x0{r};    %robot stays in initial point
    end
end
