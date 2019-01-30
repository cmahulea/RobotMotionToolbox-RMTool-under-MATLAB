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
%   First version released on October, 2018.
%   Last modification October 28, 2018.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function param=rmt_milp_pn_following_setup(param)

answer = inputdlg({...
    sprintf('Weight in cost function on number of fired transitions:'),...
    sprintf('Weight in cost function on number of moving robots:'),...
    sprintf('Weight in cost function on maximum number of robots that crossed any place (for collision avoidance)'),...
    sprintf('Number of runs in Buchi'),...
    },...
    'Robot Motion Toolbox',...
    [1;1;1;1],{num2str(param.alpha,3),num2str(param.beta,3),...
    num2str(param.gamma,3),num2str(param.kappa,3)});
if (isempty(answer))
    return;
end
%alpha
input_val = char(answer{1});
todoOK = rmt_detect_error(input_val,0,1000);
if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid range for alpha is betweeen 0 and 1000!'),'Robot Motion Toolbox','modal'));
    error('Valid range for alpha is betweeen 0 and 1000!');
else
    param.alpha = eval(input_val);
end

%beta
input_val = char(answer{2});
todoOK = rmt_detect_error(input_val,0,1000);
if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid range for beta is betweeen 0 and 1000!'),'Robot Motion Toolbox','modal'));
    error('Valid range for beta is betweeen 0 and 1000!');
else
    param.beta = eval(input_val);
end

%gamma
input_val = char(answer{3});
todoOK = rmt_detect_error(input_val,0,1000);
if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid range for gamma is betweeen 0 and 1000!'),'Robot Motion Toolbox','modal'));
    error('Valid range for gamma is betweeen 0 and 1000!');
else
    param.gamma = eval(input_val);
end

%kappa
input_val = char(answer{4});
todoOK = rmt_detect_error(input_val,1,10);
if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid range for kappa is betweeen 1 and 10!'),'Robot Motion Toolbox','modal'));
    error('Valid range for kappa is betweeen 1 and 10!');
else
    param.kappa = eval(input_val);
end


return;
