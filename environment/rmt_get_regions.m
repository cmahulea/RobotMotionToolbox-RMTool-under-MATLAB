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

function [objects,initial_points,final_points] = rmt_get_regions(mission_task)
% let the user to introduce the regions of interest or generate random
% these regions
% mission_task is 1 for ltl formula or 2 for boolean task

if (mission_task == 0) %reachability task -> ask for obstacles
    prompt = {'Number of obstacles:'};
    dlg_title = 'Obstacles';
    num_lines = 1;
    defaultans = {'3'};
    input_user = inputdlg(prompt,dlg_title,num_lines,defaultans);
    if isempty(input_user)
        return;
    end
    reg_no = char(input_user(1));
    if isempty(reg_no)
        return;
    end
    try
        reg_no = str2double(reg_no);
    catch
        uiwait(errordlg(sprintf('\nNumber of obstacles should be a natural number between 1 and 5!'),'Robot Motion Toolbox','modal'));
        error('\nNumber of obstacles should be a natural number between 1 and 5!');
    end
else
    prompt = {'Number of regions of interest:','Number of robots'};
    dlg_title = 'Robot Motion Toolbox';
    num_lines = 1;
    defaultans = {'3','2'};
    input_user = inputdlg(prompt,dlg_title,num_lines,defaultans);
    if isempty(input_user)
        return;
    end
    reg_no = char(input_user(1));   % Reading of region's numbers from input interface
    robot_no = char(input_user(2)); % Reading of robot's numbers from input interface
    if (isempty(reg_no) || isempty(robot_no))
        return;
    end
    try
        reg_no = str2double(reg_no);
        robot_no = str2double(robot_no);
    catch
        uiwait(errordlg(sprintf('\nNumber of regions and of robots should be a natural number!'),'Robot Motion Toolbox','modal'));
        error('Number of regions and of robots should be a natural number!');
    end
end
data = get(gcf,'UserData');
limits = data.frame_limits;
%we clean the workspace axes
rmt_delete_axes(data,mission_task);

% Menu to create the Environment
% 1 Mode Manual
% 2 Mode Random
choiceMenuEnv = questdlg('How do you want to generate the environment?', ...
    'Robot Motion Toolbox', ...
    'Manual','Random','Yes');

% Mode Manual
if(strcmpi(choiceMenuEnv,'Manual'))
    if (mission_task == 0)%reachability task
        [objects,initial_points,final_points] = rmt_define_regions([limits(1),limits(2),limits(3),limits(4)],reg_no, data.handle_env,1,1);
    else %ltl or boolean task
        [objects,initial_points,~] = rmt_define_regions([limits(1),limits(2),limits(3),limits(4)],reg_no, data.handle_env,robot_no,0);
        final_points={};
    end
else
    % Creation Enviroment with Random Function
    bound=[limits(1),limits(2),limits(3),limits(4)];
    color=data.reg_plot.color_full;
    %Set dimension of region of interest
    prompt = {'Set dimension of region of interest'};
    dlg_title = 'Robot Motion Toolbox';
    num_lines = 1;
    default_size = {'1'};
    size_obs = inputdlg(prompt,dlg_title,num_lines,default_size); % Reading a dimension of interest's region from input interface
    sizeObs= str2double(size_obs{1,1}); %Converiosn char in number
    if isempty(size_obs)
        sizeObs = 1;
    end
    if (mission_task == 0)%reachability task
        [objects,initial_points,final_points]=rmt_random_env(data.handle_env,reg_no,sizeObs,bound,1,1,limits,color);
    else %ltl or boolean task
        [objects,initial_points,final_points]=rmt_random_env(data.handle_env,reg_no,sizeObs,bound,0,robot_no,limits,color);
    end
end
return;