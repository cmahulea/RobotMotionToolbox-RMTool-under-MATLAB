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

function rmt_path_planning_boolean_new
%Path-planning with boolean specificaion and Petri net models (mathematical programming approach)
%Collisions will be considered as hard constraints

data = get(gcf,'UserData');
%Takes the string containing the Boolean formula inserted
data.Bool_formula = get(findobj(gcf,'Tag','booleanformula'),'String');
tic;
[Pre,Post] = rmt_construct_PN(data.T.adj);
time1 = toc;
m0=data.T.m0;
N_r = length(data.RO);

%construct the constraints for the computation of the intermediate marking
%to satisfy the Boolean formula on trajectory
tic;
[A,b,Aeq,beq,cost,obstacles] = rmt_construct_constraints_intermediate(Pre,Post,m0,data.T.props,data.Bool_formula);
time = toc;
message = sprintf('Petri net system has %d places and %d transitions\nTime spent for creating it: %g secs', size(Pre,1),size(Pre,2),time1);
message = sprintf('%s\nThe MILP for intermediate state to satify the formula on trajectory has %d variables and %d equality constraints and %d inequality constraints;\nTime spent for creating the problem: %g secs\n', message, size(A,2), size(Aeq,1), size(A,1),time);
uiwait(msgbox(message,'Robot Motion Toolbox','modal'));

data.Pre = Pre;
data.Post = Post;
set(gcf,'UserData',data);%to save data
nplaces = size(Pre,1);
ntrans = size(Pre,2);

tic;
[xmin, message] = rmt_path_planning_boolean_milp(cost,A,b,Aeq,beq,nplaces,ntrans,N_r+1,length(data.T.props),message); %solve the milp problem
time = toc;

message2 = sprintf('\nTime of solving the MILP for computing the intermediate marking to satisfy the formula on trajectory: %g secs\n', time);
message = sprintf('%s%s', message, message2);
%uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));

message = sprintf('%s\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0>eps*10^5)),mat2str(m0(m0>eps*10^5)));
Sync_m = []; %synchronization from m0 to intermediate marking where the formula on trajectory is satisfied
m0_robot = rmt_marking2places(m0);
Run_cells = m0_robot;

[message,Run_cells,Sync_m] = rmt_path_planning_boolean_trajectories(xmin,N_r+1,Pre,Post,Run_cells,m0_robot,message);

message = sprintf('%sBoolean variables in solutions for the intermediate state are: %s\n',message,mat2str(xmin((N_r+1)*(size(Pre,1)+size(Pre,2))+1:end)));

%%%%%%%%%%%compute trajectories from intermediate marking to the final one

m0= zeros(nplaces,1);
m0(Run_cells(:,end))=1;
tic;
%construct the constraints for the computation of the intermediate marking
%to satisfy the Boolean formula on trajectory
[A,b,Aeq,beq,cost] = rmt_construct_constraints_final(Pre,Post,m0,data.T.props,data.Bool_formula, obstacles);
message = sprintf('%s\nThe MILP for the final state computation has %d variables and %d equality constraints and %d inequality constraints;\nTime spent for creating the problem: %g secs\n', message, size(A,2), size(Aeq,1), size(A,1),toc);
%uiwait(msgbox(message,'Robot Motion Toolbox','modal'));

tic;
[xmin, message] = rmt_path_planning_boolean_milp(cost,A,b,Aeq,beq,nplaces,ntrans,sum(m0)+2,length(data.T.props),message); %solve the milp problem
time = toc;

message2 = sprintf('\nTime of solving the MILP for computing the final marking to satisfy the formula on the final state: %g secs\n', time);
message = sprintf('%s%s', message, message2);
%uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));

message = sprintf('%s\nIntermediate marking [ %s ] = %s\n',message,mat2str(find(m0>eps*10^5)),mat2str(m0(m0>eps*10^5)));
m0_robot = Run_cells(:,end); %start from where we arrived
Sync_m2 = []; %synchronization from intermediate to final marking

[message,Run_cells,Sync_m2] = rmt_path_planning_boolean_trajectories(xmin,N_r+2,Pre,Post,Run_cells,m0_robot,message);

message = sprintf('%sBoolean variables for the final state in solutions are: %s\n',message,mat2str(xmin((N_r+2)*(size(Pre,1)+size(Pre,2))+1:end)));
message = sprintf('%sRequired number of synchronizations: %d\n',message,length(Sync_m)+length(Sync_m2));

%%Execution monitoring strategy starts from here:

rob_traj = rmt_rob_cont_traj_new(data.T,Run_cells,data.initial);    %continuous trajectory of each robot
data.trajectory = rob_traj;


cla(data.handle_env);
rmt_represent_atomic_props(data.T.Vert,data.propositions);    %represent all atomic props and store handles

data.rob_plot.line_color = {'r','b','m','g','c','k','y',[0.8500 0.3250 0.0980],[0.4940 0.1840 0.5560],[0.6350 0.0780 0.1840],[0 0.4470 0.7410]};

% choiceMenu = questdlg(sprintf('A solution for robot trajectories was found. Based on the returned trajectories and the order of the robot, do you want to improve the solution by releasing in a dynamic manner the common cells of the paths?'), ...
%     'Robot Motion Toolbox - Path planning with dynamic release of common cells','All common cells','Common cells for paths of R_i, R_[i+1]','No','No');
% if strcmpi(choiceMenu,'All common cells')
%     % improved trajectories based on dynamic resource release
%     [new_run_cells, pos_rob, message] = rmt_path_planning_dyn_release_resources(Run_cells, rob_traj, data, 'all',message);
% elseif strcmpi(choiceMenu,'Common cells for paths of R_i, R_[i+1]')
%     [new_run_cells, pos_rob, message] = rmt_path_planning_dyn_release_resources(Run_cells, rob_traj, data, 'pairs',message);
% else % trajectories given by the result from CM & MK 2020 (Bool spec)
    message = sprintf('%s\nSOLUTION - runs of robots: \n',message);
    for j = 1 : size(Run_cells,1)
        message = sprintf('%s\nRobot %d: ',message,j);
        temp = Run_cells(j,:);
        for k = 1 : length(temp)-1
            message = sprintf('%s%d,',message,temp(k));
        end
        message = sprintf('%s%d',message,temp(length(temp)));
    end
    for r=1:length(rob_traj)    %plot trajectories of robots
        plot(rob_traj{r}(1,1),rob_traj{r}(2,1),'Color',data.rob_plot.line_color{r},...
            'Marker',data.rob_plot.marker{r},'LineWidth',data.rob_plot.line_width{r});
        plot(rob_traj{r}(1,:),rob_traj{r}(2,:),'Color',data.rob_plot.line_color{r},...
            'LineWidth',data.rob_plot.line_width{r});
        plot(rob_traj{r}(1,end),rob_traj{r}(2,end),'Color',data.rob_plot.line_color{r},...
            'Marker',data.rob_plot.marker{r},'LineWidth',data.rob_plot.line_width{r},'Color','r');
    end

    for r=1:length(rob_traj)    %plot trajectories of robots
        for tt = 1 : length(Sync_m)
            plot(rob_traj{r}(1,Sync_m{tt}),rob_traj{r}(2,Sync_m{tt}),'Color',data.rob_plot.line_color{r},...
                'Marker',data.rob_plot.marker{r},'LineWidth',data.rob_plot.line_width{r},'Color','b');
        end
    end

    for r=1:length(rob_traj)    %plot trajectories of robots
        for tt = 1 : length(Sync_m2)
            plot(rob_traj{r}(1,Sync_m2{tt}),rob_traj{r}(2,Sync_m2{tt}),'Color',data.rob_plot.line_color{r},...
                'Marker',data.rob_plot.marker{r},'LineWidth',data.rob_plot.line_width{r},'Color','b');
        end
    end

% end


button = questdlg('Save the details to a text file?','Robot Motion Toolbox');
if strcmpi(button,'Yes')
    [filename, pathname] = uiputfile('*.txt', 'Save experiments as');
    fileID = fopen(fullfile(pathname, filename),'w');
    fprintf(fileID,'%s',message);
    fclose(fileID);
end

set(gcf,'UserData',data);%to save data
