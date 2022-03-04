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

function [at_pr_h] = rmt_represent_atomic_props(C,propositions)
%atomic propositions are unions (sets) of cells from partition

data = get(gcf,'UserData');
for i = 1 : length(data.reg_plot.text_cells_h)
    if ishandle(data.reg_plot.text_cells_h(i))
        delete(data.reg_plot.text_cells_h(i))
    end
end
visible = get(data.menuViewLabels,'Checked');
dummy_cells = setdiff(1:length(C),cell2mat(propositions)); %handles to cells that are not atomic propositions

name_cells = strcat(data.label_cells.name, '_{%d}');

at_pr_h=cell(1,length(propositions));   %handles for each atomic prop
text_cells_h = zeros(1,length(C));
%represent cells
for i=1:length(C)
    fill(C{i}(1,:),C{i}(2,:),'w','FaceAlpha',0);
    %write cell numbers to cells that are not atomic propositions
    if ~isempty(intersect(dummy_cells,i))
        centr=mean(C{i},2)';
        text_cells_h(i) = text(centr(1),centr(2),sprintf(name_cells,i),'HorizontalAlignment','center','Color','k','Visible',visible,'FontSize',data.label_cells.size);
    end
end

%represent atomic propositions
colors=data.reg_plot.color;

for i=1:length(propositions)
    at_pr_h{i}=zeros(1,length(propositions{i}));
    for j=1:length(propositions{i})
        cell_ind=propositions{i}(j);    %index of current cell
        if mod(i,8) == 0 % for the colors orage and purple (index 8 and 9), which don't have a shortcut in matlab - can be represented only as RGB code
             at_pr_h{i}(j) = fill(C{cell_ind}(1,:),C{cell_ind}(2,:),[0.8500 0.3250 0.0980],'LineStyle','-.','FaceAlpha',0.4,'EdgeColor',[0.8500 0.3250 0.0980]);
        centr=mean(C{cell_ind},2)';
        text_cells_h(cell_ind) = text(centr(1),centr(2),sprintf(name_cells,cell_ind),'HorizontalAlignment','center','Color','k','Visible',visible,'FontSize',data.label_cells.size);
        elseif mod(i,9) == 0 
             at_pr_h{i}(j) = fill(C{cell_ind}(1,:),C{cell_ind}(2,:),[0.4940 0.1840 0.5560],'LineStyle','-.','FaceAlpha',0.4,'EdgeColor',[0.4940 0.1840 0.5560]);
        centr=mean(C{cell_ind},2)';
        text_cells_h(cell_ind) = text(centr(1),centr(2),sprintf(name_cells,cell_ind),'HorizontalAlignment','center','Color','k','Visible',visible,'FontSize',data.label_cells.size);
        else
        
        at_pr_h{i}(j) = fill(C{cell_ind}(1,:),C{cell_ind}(2,:),colors(i),'LineStyle','-.','FaceAlpha',0.4,'EdgeColor',colors(i));
        centr=mean(C{cell_ind},2)';
        text_cells_h(cell_ind) = text(centr(1),centr(2),sprintf(name_cells,cell_ind),'HorizontalAlignment','center','Color','k','Visible',visible,'FontSize',data.label_cells.size);
        end
    end
end

message = sprintf('REGIONS OF INTEREST:');
for i = 1 : length(propositions)
    temp = sprintf('- Output y_{%d} (%s) is for O_{%d} = \\{',i,data.reg_plot.color_full{i},i);
    for j = 1 : length(propositions{i})-1
        temp = sprintf('%sp_{%d}, ',temp,propositions{i}(j));
    end
    temp = sprintf('%sp_{%d}\\}',temp,propositions{i}(length(propositions{i})));       
    message = sprintf('%s\n%s',message,temp);
end

set(data.handle_text,'String',message,'Position',[0.35    0.054    0.63    0.2477]);
data.reg_plot.at_pr_h = at_pr_h;
data.reg_plot.text_cells_h = text_cells_h;
set(gcf,'UserData',data);
set(gca,'Box','off');
set(gca,'XTick',0:2:16,'YTick',0:2:10);
