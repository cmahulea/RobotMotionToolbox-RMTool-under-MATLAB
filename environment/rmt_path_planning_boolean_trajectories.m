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
%   First version released on November, 2019. 
%   Last modification November 2019.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function [message,Run_cells,Synchron] = rmt_path_planning_boolean_trajectories(xmin,intermediate_markings,Pre,Post,Run_cells,m0_robot,message);

Synchron = [];
nplaces = size(Pre,1);
ntrans = size(Pre,2);
for i = 1 : intermediate_markings  %for all intermediate markings
    m = xmin((i-1)*(nplaces+ntrans)+1 : (i-1)*(nplaces+ntrans)+nplaces);
    fire = xmin((i-1)*(nplaces+ntrans)+nplaces+1 : (i-1)*(nplaces+ntrans)+nplaces+ntrans);
    if (max(fire) > eps*10^5)
        if (i > 1)
            Synchron{i-1} = size(Run_cells,2); %syncronization
        end
        message = sprintf('%s\n============STEP %d =============\n',message,i);
        message=sprintf('%s\nMarking [ %s ] = %s\n',message,mat2str(find(m>eps*10^5)),mat2str(m(m>eps*10^5)));
        message = sprintf('%s\nSigma [ %s ] = %s\n',message,mat2str(find(fire>eps*10^5)),mat2str(fire(fire>eps*10^5)));
        %%compte the intermediate markings by firing sigma=fire
        while (sum(fire)>eps*10^5)
            %find if each robot change its regions
            m_robot = zeros(length(m0_robot),1);
            for j = 1 : length(m0_robot)
                %find output transition of robot k
                outputTrans = find(Pre(m0_robot(j),:)>eps*10^5);
                %see if one of these trnsitions may fire
                for k = 1 : length(outputTrans)
                    if (fire(outputTrans(k)) > eps*10^5)
                        %trnasition is firing
                        m_robot(j) = find(Post(:,outputTrans(k))>eps*10^5);
                        fire(outputTrans(k)) = 0;
                        break;
                    end
                end
                if (m_robot(j) == 0) %robot j i snot moving
                    m_robot(j) = m0_robot(j);
                end
            end
            Run_cells = [Run_cells, m_robot];
            m0_robot = m_robot;
        end
    end
end
Synchron{end+1} = size(Run_cells,2);
