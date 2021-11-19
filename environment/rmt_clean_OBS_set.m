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

function OBS_set = rmt_clean_OBS_set(Nobstacles, formula, OBS_set)
%clean the possible set of observation for one robot, based on the regions
%captured in formula

%   inputs:
% formula: string containing LTL formula (all its atomic proposition are in alphabet from which alphabet_set was created) (use spaces in formula)
%atomic propositions are of form y0...0x, where x is an integer; true, false
% USE "y" for denoting propositions (outputs of system)
%Boolean operators: ! - negation; & - and; | - or; -> - implication; <-> - equivalence;
%Temporal operators: U - until; R - release; F - eventually; G - always; X - next
%example of formula: (F y1) & G !(y2 | y3)
% Nobstacles: number of obstacles in the environment
% OBS_set - the possible set of observation of one robot

%   output:
% OBS_set - the possible set of observation of one robot
% -----------------------------------------------------------

%make changes in formula string:
formula = regexprep(formula,'[pPoOY]','y'); %if user mistakenly uses for props letters p, P, o, O, or Y -> replace with lower-case "y"
numbers = regexp(formula, '\d*', 'Match'); %extract the numbers
reg_formula = unique(cellfun(@str2num, numbers));

all_reg = 1:Nobstacles; 
useless_reg = find(ismember(all_reg, reg_formula) == 0); % save the regions which are not used in LTL formula
aux_OBS_set = [];
k = 1;
for i = 1:size(OBS_set,1)
    if(isempty(find(ismember(useless_reg,OBS_set(i,:)),1)))
        aux_OBS_set = [aux_OBS_set; OBS_set(i,:)];
        k = k + 1;
    end
end

OBS_set = aux_OBS_set;