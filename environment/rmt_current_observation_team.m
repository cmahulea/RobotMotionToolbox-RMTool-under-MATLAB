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

function c_obs = rmt_current_observation_team(Obs,propositions,simplices)

N_p=length(propositions);  %number of proposititons
props_obs=[];   %observed propositions at current state

for i=1:N_p
    if ~isempty(intersect(simplices,propositions{i}))  %in proposition i at least one occupied simplex belongs to its set of regions
        props_obs=[props_obs, i]; %props_obs will be sorted
    end
end

if isempty(props_obs)   %no proposition in current region
    props_obs=N_p+1;    %dummy symbol for the environment
end

props_obs=[props_obs , zeros(1, N_p-length(props_obs))];    %pad with zeros until length props_obs

[~,c_obs]=ismember(props_obs,Obs,'rows');
        