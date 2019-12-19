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
%   Last modification December 11, 2019.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function rmt_redistribute_robots

data = get(gcf,'UserData');

free_regions = data.T.Q;
for i = 1 : length(data.T.props)
    free_regions = setdiff(free_regions,data.T.props{i});
end

for i = 2 : length(data.T.RO)
    robot = data.T.RO(i);
    if ~isempty(intersect(data.T.RO(1:i-1),robot))%robot i in a region occupied by other robot
        new_index = randi(length(free_regions),1,1);
        data.T.RO(i) = free_regions(new_index);
        data.initial{i} = mean(data.T.Vert{data.T.RO(i)},2)';
        data.final{i} = mean(data.T.Vert{data.T.RO(i)},2)';
        free_regions(new_index)=[];
    end
end

data.RO = data.T.RO;
data.T.m0 = zeros(length(data.T.m0),1);
data.T.m0(data.T.RO) = 1;
set(gcf,'UserData',data);
