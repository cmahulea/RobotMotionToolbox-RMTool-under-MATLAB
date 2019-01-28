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

function Obs = rmt_observation_set(N_p)
%construct possible set of observations (dummy-free proposition will be the last one)
%N_p - number of propositions (excepting the dummy one-environment) that can be simultaneously satisfied 
%consider maximum number of possible simultaneous observations (assume more robots or/and that all props. can overlap)
% Obs will correspond to the power set of 1:N_p
% Obs will be a matrix with 2^N_p rows and N_p columns

temp_obs=1:N_p;   %atomic propositions, for constructing possible observations (without dummy prop)
temp_cell=mat2cell( repmat(temp_obs,N_p,1) , ones(1,N_p) , length(temp_obs) );  %duplicate observables of transition systems
temp_obs=rmt_cartesian_product(temp_cell{:});  %possible observables, on rows (more robots can have the same observables, that's why we use carth product); observables will be labeled with indices of rows (in T.obs)
temp_obs=unique(sort(temp_obs,2),'rows');   %sort rows and remove identical ones (they would have the same observable)
for i=1:size(temp_obs,1)  %modify observables (keep at most one occurence of same prop on each row, and pad with zeros until length 
    obs=unique(temp_obs(i,:));    %keep unique elements on each row, and pad wih zeros
    if length(obs)<N_p %pad with zeros until number of propositions
        obs((end+1):N_p)=0;
    end
    temp_obs(i,:)=obs;
end
temp_obs=unique(temp_obs,'rows');   %again remove identical rows (there may appear new repetitions, due to keeping only unique elements on each row and padding with zeros)
%until now temp_obs has 2^n-1 lines (-1 for the empty set); add a line for the dummy prop (we shouldn't add dummy to other satisfied props, only on a single line)
temp_obs(end+1,:)=[N_p+1 , zeros(1,N_p-1)]; %dummy has index (N_p+1), and pad with zeros after it
Obs=temp_obs; %Obs contains possible observables of the system (on rows, propositions that can be satisfied; last row for dummy)
return
