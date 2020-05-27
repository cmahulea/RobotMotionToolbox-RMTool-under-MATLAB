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
%   First version released on November, 2018.
%   Last modification November 10, 2018.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function rmt_path_planning_ltl_trsys
%Path-planning with LTL specificaion and transition system models (graph based approach)

data = get(gcf,'UserData');
data.formula = get(findobj(gcf,'Tag','ltlformula'),'String');

probability=ones(1,length(data.propositions));
choice1_1 = questdlg('Choose robot model to use:', ...
    'Robot Motion Toolbox', ...
    'Full','Equiv. rob. permutations (as reach. graph)','Collapsed robot model','Full');
switch choice1_1
    case 'Full'
        choice1_1=2;
        data.hwait = waitbar(0,'Computing trajectories (Transition System full). Please wait...','Name','Robot Motion Toolbox',...
            'WindowStyle','modal','CloseRequestFcn','rmt(''close_hwait'')');
    case 'Equiv. rob. permutations (as reach. graph)'
        data.hwait = waitbar(0,'Computing trajectories (Transition System reduced w.r.t. robot permutations). Please wait...','Name','Robot Motion Toolbox',...
            'WindowStyle','modal','CloseRequestFcn','rmt(''close_hwait'')');
        choice1_1=1;
    case 'Collapsed robot model' 
        data.hwait = waitbar(0,'Computing trajectories (Transition System reduced). Please wait...','Name','Robot Motion Toolbox',...
            'WindowStyle','modal','CloseRequestFcn','rmt(''close_hwait'')');
        choice1_1=0;
        tic;
        [T_red,propositions_red,RO_red] = rmt_reduce_T(data.T,data.propositions,data.RO);
        time_reduce=toc;
    otherwise
        error('\nERROR - Undefined/wrong case for Robot model to use!\n');
end
set(gcf,'UserData',data);%to save data
    
if choice1_1==2
    tic;
    Tg = rmt_tr_sys_obs_team(data.T,data.RO,data.propositions,probability);    %compute team (global) transition system, including probabilities of observing propositions
    message = sprintf('Model of a robot (full transition system) - has %d states\nTeam model (global) - full transition system - has %d states\nTime spent for creating it: %g secs', length(data.T.Q),length(Tg.Q), toc);
elseif choice1_1==1
    tic;
    Tg = rmt_tr_sys_team_reduced(data.T,data.RO,data.propositions,probability); %team model, reduced w.r.t. robot permutations (as a reachability graph of PN)
    message = sprintf('Model of a robot (full transition system) has %d states\nTeam model (global) - reduced based on robot permutations (as reachability graph) - has %d states\nTime spent for creating it: %g secs', length(data.T.Q),length(Tg.Q), toc);
else %collapsed model
    tic;
    Tg = rmt_tr_sys_obs_team(T_red,RO_red,propositions_red,probability);    %compute team (global) transition system, including probabilities of observing propositions
    message = sprintf('Model of a robot (reduced transition system) - has %d states\nReduced model constructed in %g secs\nTeam model (global) - product of reduced systems - has %d states\nTime spent for creating it: %g secs', length(data.T.Q),time_reduce,length(Tg.Q), toc);
end

data.Tg=Tg;
set(gcf,'UserData',data);

% Control of regions of interest in the LTL Formula
regionFormula=strfind(data.formula, 'u');
if(data.Nobstacles < size(regionFormula,2))
    uiwait(msgbox('LTL Formula is not correct. The number of proposition and region of interest is not equal. Please re-insert!','Robot Motion Toolbox','modal'));
    prompt = {'New LTL Formula:'};
    dlg_title = 'Robot Motion Toolbox';
    num_lines = 1;
    defaultans = {''};
    %defaultans = {'(F u1) & G !(u2 | u3)'};
    input_user = inputdlg(prompt,dlg_title,num_lines,defaultans);
    data.formula= char(input_user(1));   % Reading of region's numbers from input interface
    B = rmt_create_buchi(data.formula, Tg.Obs);
    data.B=B;
else
    B = rmt_create_buchi(data.formula, Tg.Obs);
    data.B=B;
end
    
set(gcf,'UserData',data);
    tic;
    Pg = rmt_product_autom_prob_team(Tg,B);  %Pg has trans with log of probabilities plus epsilon (used for Dijkstra), and 2 additional costs: probability of transition and number of moving robots
    message2 = sprintf('\nBuchi automaton has %d states;\nProduct automaton has %d states;\nTime spent for creating it: %g secs', size(B.S,2), size(Pg.S,1), toc);
    message = sprintf('%s%s', message, message2);
data.Pg = Pg;

tic;
[run_Tg,~,~,path_Tg,path_B,~] = rmt_find_accepted_run_multicost(Pg,'prob','move');  %solution in Pg and projection to Tg and B

data = get(gcf,'UserData');
delete(data.hwait);
set(gcf,'UserData',data);%to save data

if isempty(run_Tg)
    message2 = sprintf('\nNo solution foundf! The formula seems not be possible to fulfill');
    message = sprintf('%s%s', message, message2);
    return;
else
    message2 = sprintf('\nTime for finding accepted run: %g secs', toc);
    message = sprintf('%s%s', message, message2);
end
if choice1_1==2 %full model
    [~,R_paths,R_trajs,~] = rmt_robot_trajectory_team(data.T,Tg,run_Tg,path_Tg);  %each robot starts from centroid of its initial cell; R_trajs is a cell array,
elseif choice1_1==1 %reduced model w.r.t. permutations (as a reachability graph of PN)
%     [~,R_paths,R_trajs,~] = rmt_robot_trajectory_team_red_permut(data.T,Tg,Tg_init,run_Tg,path_Tg);
    [~,R_paths,R_trajs,~] = rmt_robot_trajectory_team_reduced(data.T,data.RO,Tg,run_Tg,path_Tg);
else %reduced model by collapsing states in robot model
    [~,R_paths,R_trajs,~,~] = rmt_robot_trajectory_team_collapsed(T_red,data.T,data.RO,Tg,run_Tg,path_Tg,path_B);
end
%we clean the workspace figure
data.trajectory = R_trajs;
set(gcf,'UserData',data);
cla(data.handle_env);
rob_plot = data.rob_plot;
%%Execution monitoring strategy starts from here:
N_r = length(data.RO);
current_pos=cell(1,N_r);
for r=1:N_r
    current_pos{r}=data.initial{r};%R_trajs{r}(:,1); %current_pos - initial (current) position of robots (cell, current_pos{i} is a column vector with 2 rows)
end
rmt_represent_atomic_props(data.T.Vert,data.propositions);    %represent all atomic props and store handles
for r=1:N_r
    %plot(current_pos{r}(1),current_pos{r}(2),rob_marker{r});  %position where robot waits
    plot(current_pos{r}(1),current_pos{r}(2),'Color',rob_plot.line_color{r},'LineStyle',rob_plot.line{r},'LineWidth',rob_plot.line_width{r},'Marker',rob_plot.marker{r},'MarkerFaceColor',rob_plot.face_color{r});
end
i = 1;
while i<=size(R_paths,2)  %go through synchronized paths and simulate robot moves and appearance/disappearance of propositions
    %                        fprintf('\n\n Current team state (simplices) = [%s]'' ; press any key to test cases.',num2str(R_paths(:,i)'));
    %c_obs= rmt_current_observation_team(Tg.Obs,data.propositions,R_paths(:,i));   %current observation of Tg (row index in Tg.Obs) - current simplices R_paths(:,i), and appeared/dissapeared regions are given by app_props
    for r=1:N_r
        %plot([current_pos{r}(1),R_trajs{r}(1,i+1)],[current_pos{r}(2),R_trajs{r}(2,i+1)],rob_marker{r},'LineWidth',2)    %plot trajectory until next cell
        plot([current_pos{r}(1),R_trajs{r}(1,i+1)],[current_pos{r}(2),R_trajs{r}(2,i+1)],'Color',rob_plot.line_color{r},'LineStyle',rob_plot.line{r},'LineWidth',rob_plot.line_width{r},'Marker',rob_plot.marker{r},'MarkerFaceColor',rob_plot.face_color{r});
        current_pos{r} = R_trajs{r}(:,i+1);   %update current robot position (on boundary of next cell)
    end
    i=i+1;
end

button = questdlg('Save the details to a text file?','Robot Motion Toolbox');
if strcmpi(button,'Yes')
    [filename, pathname] = uiputfile('*.txt', 'Save experiments as');
    fileID = fopen(fullfile(pathname, filename),'w');
    fprintf(fileID,'%s',message);
    fclose(fileID);
end

set(gcf,'UserData',data);%to save data
