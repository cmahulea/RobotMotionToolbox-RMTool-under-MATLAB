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

function param=rmt_milp_pn_boolean_setup(param)

answer = inputdlg({...
    sprintf('Weight in cost function on number of fired transitions:'),...
    sprintf('Weight in cost function on soft contraints (of var. b) (for collision avoidance):'),...
    sprintf('Number of PN intermediate markings'),...    
    sprintf('Number of robots which waits to enter a common cell'),...
    },...
    'Robot Motion Toolbox',...
    [1;1;1;1],{num2str(param.lambda,3),num2str(param.mu,3),num2str(param.kappa,3),num2str(param.UserCount,3)});
if (isempty(answer))
    return;
end
%lambda
input_val = char(answer{1});
todoOK = rmt_detect_error(input_val,0,1000);
if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid range for \\lambda is betweeen 0 and 1000!'),'Robot Motion Toolbox','modal'));
    error('Valid range for alpha is betweeen 0 and 1000!');
else
    param.lambda = eval(input_val);
end

%mu
input_val = char(answer{2});
todoOK = rmt_detect_error(input_val,0,1000);
if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid range for \\mu is betweeen 0 and 1000!'),'Robot Motion Toolbox','modal'));
    error('Valid range for beta is betweeen 0 and 1000!');
else
    param.mu = eval(input_val);
end

%kappa
input_val = char(answer{3});
todoOK = rmt_detect_error(input_val,1,50);
if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid range for number of intermediate markings is betweeen 1 and 50!'),'Robot Motion Toolbox','modal'));
    error('Valid range for kappa is betweeen 1 and 50!');
else
    param.kappa = eval(input_val);
end

%UserCount
input_val = char(answer{1});
todoOK = rmt_detect_error(input_val,0,1000);
if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid range for UserCount is between 0 and 20!'),'Robot Motion Toolbox','modal'));
    error('Valid range for UserCount is betweeen 0 and 20!');
else
    param.UserCount = eval(input_val);
end

return;
