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
%   Last modification December 13, 2019.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================
function new_robot_regions = rmt_find_next_state(m,m0,robot_regions,sigma,Post,Pre)
%function that returns the positions of the robots after firing sigma. the
%initial marking is m0 and the final one is m.

if ~isempty(setdiff(find(m0>eps*10^5),robot_regions)) || ~isempty(setdiff(robot_regions,find(m0>eps*10^5)))
    error('Regions in m0 and robot_regions are not igual');
end
new_robot_regions = robot_regions;

while (sum(sigma) > eps*10^5);
    fired_transitions = find(sigma > eps*10^5);
    inputPlace = find(Pre(:,fired_transitions(1)));
    outputPlace = find(Post(:,fired_transitions(1)));
    robots = find(robot_regions == inputPlace);
    robot_regions(robots(1)) = -inf;
    new_robot_regions(robots(1)) = outputPlace;
    sigma(fired_transitions(1)) = sigma(fired_transitions(1)) - 1;
end

if ~isempty(setdiff(find(m>eps*10^5),new_robot_regions)) || ~isempty(setdiff(new_robot_regions,find(m>eps*10^5)))
    error('Something happened');
end


% for i = 1 : length(robot_regions)
%     if (m(robot_regions(i))> eps*10^5 &&  m0(robot_regions(i))> eps*10^5) %robot robot_regions(i) is not moving from m0 to m
%        new_robot_regions(i) = robot_regions(i);
%        m(robot_regions(i)) = m(robot_regions(i)) - 1;
%        m0(robot_regions(i)) = m0(robot_regions(i)) - 1;
%     elseif (m(robot_regions(i)) < eps*10^5 &&  m0(robot_regions(i)) > eps*10^5)%robot robot_regions(i) moves
%         outputTrans = find(Pre(robot_regions(i),:)); %output transitions of place robot_regions(i)from where moved the robot
%         for j = length(outputTrans): -1 : 1
%             if (sigma(outputTrans(j)) < eps*10^5)  %transition i snot fired in sigma
%                 outputTrans(j) = [];
%             end
%         end
%         for j = 1 : length(outputTrans)
%             outputPlace = find(Post(:,outputTrans(j))); %find the possible region of the robot by firing outputTrans(j)
%             if (m(outputPlace) > eps*10^5) %transition can be fired and we force to fire
%                 new_robot_regions(i) = outputPlace;
%                 m(outputPlace) = m(outputPlace) - 1;
%                 m0(outputPlace) = m0(outputPlace) - 1;
%                 sigma(outputTrans(j)) = sigma(outputTrans(j)) - 1;
%             end
%         end
%     end
% end
        