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

function [Obs, NotObs] = rmt_observation_set_new_v2(OBS_set,N_p,N_r)
%construct possible set of observations(dummy-free proposition will be the last one)
% inputs:
%   OBS_set - the possible set of observation of one robot
%   N_r - number of robots
%   N_p - number of propositions (excepting the dummy one-environment) that can be simultaneously satisfied 
% outputs:
%   Obs will be a matrix with N_p columns - for true observations yi
%    NotObs will be a matrix with N_p columns - for negated observations yj
%    
% e.g., for observation y1 & y2 & y3: Obs(i,:) = [1 2 0]; and
%        NotObs(i,:) = [0 0 1]
if (N_r == 1)
    Obs = OBS_set;
    return;
end

obs_1_robot = {};
for i = 1 : size(OBS_set,1)-1
    obs_1_robot{i} = OBS_set(i,find(OBS_set(i,:)));
end

temp_obs = [];

index = ones(1,N_r);
while 1  
    new_obs = [];
    for j = 1 : length(index)
        new_obs = [new_obs obs_1_robot{index(j)}];
    end
    new_obs = unique(sort(new_obs));
    temp_obs = [temp_obs ; new_obs zeros(1,N_p-length(new_obs))];
    if (length(unique(index)) == 1)
        if (index(1) == length(obs_1_robot))
            break;
        end
    end
    for k = length(index): -1: 1
        if (index(k) == length(obs_1_robot))
            index(k) = k;
            last = k;
            for l = k - 1 : -1 : 1
                if index(l) < length(obs_1_robot)
                    index(l) = index(l) + 1;
                    break;
                else
                    index(l) = k;
                end
            end
            for m = l+1 : last
                index(m) = index(m-1);
            end
            break;
        else
            index(k) = index(k) + 1;
            break;
        end
    end
end
temp_obs=unique(temp_obs,'rows');

temp_obs(end+1,:)=[N_p+1 , zeros(1,N_p-1)]; %dummy has index (N_p+1), and pad with zeros after it
Obs=temp_obs; %Obs contains possible observables of the system (on rows, propositions that can be satisfied; last row for dummy)


temp_not_obs = zeros(size(Obs)); % create the matrix for the inactive observations

for i =1:size(temp_obs,1)-1
        aux_not_obs = setdiff(1:N_p,temp_obs(i,:));
        temp_not_obs(i,aux_not_obs) = 1;
end
temp_not_obs(end,:) = temp_obs(end,:);
NotObs = temp_not_obs;
return;