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
%   First version released on January, 2019. 
%   Last modification January 31, 2019.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================


function [feasible_sigma, Rob_places, Rob_trans, Rob_positions] = rmt_sigma2trajectories(Pre,Post,m0,sigma,R0)
%convert a firing count vector sigma (obtained from ILP solution) to individual robot trajectories
%inputs:
%   Pre,Post,m0 - for PN model
%   sigma - firing count vector
%   R0 - vector with initial robots' positions (should be in correspondence with initial marking m0)
%outputs:
%   feasible_sigma will be 1 if sigma is feasible (all its transition counts can be fired) and 0 for unfeasible sigma (spurious solution)
%   (the next outputs are useful only when feasible_sigma is 1
%   Rob_places{i} is a vector containing sequence of places through which robot i travelled (in order in which they appear)
%   Rob_trans{i} is a vector containing sequence of transitions which robot i followed (in order in which they fire)
%   e.g.: when in Rob_places{i}(j), robot i fires transition Rob_trans{i}(j) and arrives in Rob_places{i}(j+1)
%   markingSim - reached marking
%   Rob_positions - vector with final robots' positions (in correspondence with reached marking "markingSim")

N_r = sum(m0);  %number of robots
Rob_places = cell(1,N_r);
Rob_trans = cell(1,N_r);
for i=1:N_r
    Rob_places{i}=R0(i);    %current position of robot i
    Rob_trans{i}=[];
end
Rob_positions = R0; %will contain updated positions of team (unsynchronized, just for knowing which is the first robot, second one, etc.)

%find robot trajectories
firings = sigma;    %transitions to fire
markingSim = m0;    %current marking
feasible_sigma=1; %solution is feasible if next "while" is not entered at least once (sigma is empty)
while (sum(firings)~=0) %there are still not-fired transitions from solution
    feasible_sigma=0; %assume solution not feasible
    temp = find(markingSim);    %non-empty places
    for j = 1 : length(temp)
        outputTrans = intersect(find(Pre(temp(j),:)),find(firings));    %feasible transitions to fire (from non-empty place with firings in solution)
        if ~isempty(outputTrans)    %there are feasible transitions: fire the first one
            feasible_sigma=1; %so far, solution is feasible
            
            moving_rob = find(Rob_positions==temp(j),1);    %pick first robot from place temp(j)
            Rob_trans{moving_rob}(end+1) = outputTrans(1);  %fire transition
            Rob_places{moving_rob}(end+1) = find(Post(:,outputTrans(1)));  %new robot place
            Rob_positions(moving_rob) = Rob_places{moving_rob}(end);    %update position of moved robot
            
            firings(outputTrans(1)) = firings(outputTrans(1)) - 1;  %decrease firing count
            markingSim = markingSim + Post(:,outputTrans(1))-Pre(:,outputTrans(1)); %new PN marking
            % fprintf('\n Firing t%d',outputTrans(1));
            % fprintf(1,'\n Positions of the robots: %s \n',displayRobots(markingSim));
            
            break;  %start again "while" loop
        end %nothing for else (other iteration of "for" may enter the if condition)
    end
    
    if feasible_sigma==0 %current transition cound no te taken ("if" from above "for" loop wasn't entered): currently no feasible transitions, but solution sigma was not finished
        fprintf('\n\tThe current firing count vector is spurious!\n')
        return; %exit function
    end
end
