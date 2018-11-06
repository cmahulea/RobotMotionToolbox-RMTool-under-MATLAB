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


%Kinematic model of a differential-drive mobile robot
function [position1, omega] = rmt_km_diff_drive_robot(position0,T,v_r,v_l,lv)
position1 = zeros(1,3);
position1(1) = position0(1) + T * (cos(position0(3))*((v_r+v_l)/2));
position1(2) = position0(2) + T * (sin(position0(3))*((v_r+v_l)/2));
position1(3) = position0(3) + T * ((v_l-v_r)/lv);
omega = T * ((v_l-v_r)/lv);
        
end
