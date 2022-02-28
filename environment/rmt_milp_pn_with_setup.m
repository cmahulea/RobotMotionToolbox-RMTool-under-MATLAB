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
%   First version released on May, 2020.
%   Last modification May, 2020.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function param=rmt_milp_pn_with_setup(param)

answer = inputdlg({...
    sprintf('Number of PN intermediate markings')},...
    'Robot Motion Toolbox',...
    [1],{num2str(param.interM,3)});
if (isempty(answer))
    return;
end
%intermediate markings
input_val = char(answer{1});
todoOK = rmt_detect_error(input_val,1,200);
if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid range for number of intermediate markings is betweeen 1 and 200!'),'Robot Motion Toolbox','modal'));
    error('Valid range for kappa is betweeen 1 and 200!');
else
    param.interM = eval(input_val);
end




return;
