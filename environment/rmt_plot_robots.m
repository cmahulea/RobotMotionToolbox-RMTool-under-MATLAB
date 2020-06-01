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
%   First version released on June, 2020. 
%   Last modification June, 2020.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function rmt_plot_robots

data = get(gcf,'UserData');
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
