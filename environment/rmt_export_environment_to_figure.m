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
%   First version released on February, 2019.
%   Last modification February 21, 2019.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function rmt_export_environment_to_figure

[filename, pathname] = uiputfile('*.fig', 'Save Trajectory/Workspace axes object as');
if (isequal(filename,0) || isequal(pathname,0))
    return;
end
data = get(gcf,'UserData');
Fig2 = figure;
copyobj(data.handle_env, Fig2);
temp = get(Fig2,'CurrentAxes');
set(temp, 'Units', 'normalized', 'Position', [0.1300 0.1100 0.7750 0.8150]);
hgsave(Fig2, fullfile(pathname, filename));
return