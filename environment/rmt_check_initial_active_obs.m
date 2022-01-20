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

function [message] = rmt_check_initial_active_obs(data,B,Obs,m0,Pre)
% This function checks if the initial position of the robots violate the
% LTL formula. 

not_null_obs = [];
temp_act_obst_cellsm0 = [];
for i = 1:length(data.T.props)
    not_null_obs = [not_null_obs data.T.props{i}];
    obs_m0 = intersect(find(m0),data.T.props{i});
    if ~isempty(obs_m0) %check if the initial pose of the robots activate any observation
        temp_act_obst_cellsm0 = [temp_act_obst_cellsm0 i];
    end
end
null_obs = setdiff([1:size(Pre,1)],not_null_obs);
idx_actobs = [];

% memorize the index for all active observations based on the initial position of the robots
for ii = 1:length(temp_act_obst_cellsm0)
    for jj = 1:length(Obs)
        if ~isempty(intersect(temp_act_obst_cellsm0(ii),Obs(jj,:))) 
           idx_actobs = [idx_actobs jj];
        end
    end
end

%if at least one robot is in free space, memorize the index for free space
flag_empty = 0;
if ~isempty(intersect(find(m0),null_obs))
    flag_empty = 1; % 
    idx_actobs = [idx_actobs size(Obs,1)]; 
end
idx_actobs = unique(idx_actobs);
    
flag_initposR = 1; %suppose that the initial position of the robots doesn't violate the LTL formula    
if length(B.trans{1,1}) >= length(idx_actobs)
    temp_order = {idx_actobs B.trans{1,1}'};
    ints_act = intersect(temp_order{1,1},temp_order{1,2});
else
    flag_initposR = 0; % the init position of robots can activate more observations than the ones from the self-loop
end

if B.new_trans{1,1} ~= Inf & ~isempty(B.new_trans{1,1})
    if isempty(ints_act) || ~isempty(setdiff(temp_order{1,1}, temp_order{1,2})) %the robots cannot activate any observations from the self-loop
        flag_initposR = 0;
    else 
        flag_initposR = 1;    
    end
else 
    flag_initposR = 1; 
end

if flag_initposR == 0
    uiwait(errordlg(sprintf('\nThe LTL formula is violated based on the initial position of the robots! Please change the initial positions!'),'Robot Motion Toolbox','modal'))
else
    message = sprintf('The initial positions of the robots does not violate the LTL formula\n');
end

end