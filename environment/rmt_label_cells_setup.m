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
%   First version released on May, 2021.
%   Last modification May 07, 2021.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function param=rmt_label_cells_setup(param)
% This function is responsible to setup the parameters necessary for
% labelling correctly the cells

answer = inputdlg({...
    sprintf('Font size for labelling the cells. Please select a font size between 6 and 20: '),...
    sprintf('Label of the cells (by default is denoted with "p_i"): '),...   
    },...
    'Robot Motion Toolbox',...
    [1;1],{num2str(param.size,3),num2str(param.name,3)});
if (isempty(answer))
    return;
end

%size
input_val = char(answer{1});
todoOK = rmt_detect_error(input_val,6,20);
if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid range for \\size is betweeen 6 and 20!'),'Robot Motion Toolbox','modal'));
    error('Valid range for size is betweeen 6 and 20!');
else
    param.size = eval(input_val);
end

%mu
input_val = char(answer{2});
if isletter(input_val)
    todoOK = 1;
else
    todoOK = 0;
end

if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid character for this parameter is char (letter between a and z)!'),'Robot Motion Toolbox','modal'));
    error('Valid type for this parameter is char!');
else
    param.name = input_val;
end


return;
