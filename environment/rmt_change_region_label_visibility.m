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
%   Last modification December 12, 2019.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function rmt_change_region_label_visibility

data = get(gcf,'UserData');
if strcmpi(get(data.menuViewLabels,'Checked'),'on')
    for i = 1 : length(data.reg_plot.text_cells_h)
        if ishandle(data.reg_plot.text_cells_h(i))
            set(data.reg_plot.text_cells_h(i),'Visible','off');
        end
    end
    set(data.menuViewLabels,'Checked','off');
else
    for i = 1 : length(data.reg_plot.text_cells_h)
        if ishandle(data.reg_plot.text_cells_h(i))
            set(data.reg_plot.text_cells_h(i),'Visible','on');
        else
            %write cell numbers to cells that are not atomic propositions
            centr=mean(data.T.Vert{i},2)';
            data.reg_plot.text_cells_h(i) = text(centr(1),centr(2),sprintf('c_{%d}',i),'HorizontalAlignment','center','Color','k','Visible','on');
        end
    end
    set(data.menuViewLabels,'Checked','on');
end
set(gcf,'UserData',data);