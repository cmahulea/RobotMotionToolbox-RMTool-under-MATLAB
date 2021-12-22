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

function rmt_path_planning_ltl_pn_with_buchi
%Path-planning with LTL specificaion and Petri net models (mathematical programming approach)

data = get(gcf,'UserData');
if (length(unique(data.RO)) ~= length(data.RO))
    rmt_plot_robots;
    uiwait(errordlg(sprintf('\nRobots should be initially located in different regions. Please change the initial position of the robots using Setup->Robots initial and final positions'),'Robot Motion Toolbox','modal'));
    error('Robots should be initially located in different regions. Please change the initial position of the robots using Setup->Robots initial and final positions');
end

%disp('START CREATION PETRI NET WITH LTL FORMULA');
data.formula = get(findobj(gcf,'Tag','ltlformula'),'String'); %% read LTL formula
total_time = 0;
if strcmp(get(data.optim.menuCplex,'Checked'),'on')
    solver = 'cplex';
elseif strcmp(get(data.optim.menuIntlinprog,'Checked'),'on')
    solver = 'intlinprog';
elseif strcmp(get(data.optim.menuGlpk,'Checked'),'on')
    solver = 'glpk';
else
    uiwait(errordlg(sprintf('\nUnknown MILP solver'),'Robot Motion Toolbox','modal'));
    error('Unknown MILP solver');
end
time_c = 0;
total_time_MILP = 0;
message_c = sprintf('--------------------------------------------------------------\n');
message_c = sprintf('%s---Path planning using Petri net models with Buchi included---\n',message_c);
message_c = sprintf('%s--------------------------------------------------------------\n',message_c);

disp('Computing trajectories (PN model with Buchi included). Please wait...');

tic;
data.Tr = rmt_quotient_T_new(data.T); %quotient of partition T, with fewer states (based on collapsing states with same observables in same connected component with same obs)
[Pre,Post] = rmt_construct_PN(data.Tr.adj);
m0=data.Tr.m0;
tiempo = toc;
message = sprintf('Petri net system has %d places and %d transitions\nTime spent for creating it: %g secs', size(Pre,1),size(Pre,2),tiempo);
message_c = sprintf('%sTime of generating the quotient PN model of the team: %g secs\n',message_c,tiempo);
time_c = time_c + tiempo;
nplaces_orig = size(Pre,1);
ntrans_orig = size(Pre,2);

%clean OBS_set based on the regions of interest included in LTL formula
data.T.OBS_set = rmt_clean_OBS_set(data.Nobstacles, data.formula, data.T.OBS_set);

%create the observation set
N_r = length(data.RO); %In RO there is a region that contains a token (robot)
N_p = data.Nobstacles;%number of regions of interest
[temp_obs] = rmt_observation_set_new(data.T.OBS_set,N_p,N_r); 

% Creating the automaton Buchi to be included in the global Petri Net
% Control on the number of region of interest
regionFormula=strfind(data.formula, 'u');
if(data.Nobstacles < size(regionFormula,2))
    uiwait(msgbox('LTL Formula is not correct. The number of proposition and region of interest is not equal. Please re-insert!','Robot Motion Toolbox','modal'));
    prompt = {'New LTL Formula:'};
    dlg_title = 'Robot Motion Toolbox';
    num_lines = 1;
    defaultans = {''};
    input_user = inputdlg(prompt,dlg_title,num_lines,defaultans);
    data.formula= char(input_user(1));   % Reading of region's numbers from input interface
    tic;
    B = rmt_create_buchi(data.formula, temp_obs);
else
    tic;
    B = rmt_create_buchi(data.formula, temp_obs);
end
tiempo = toc;
message = sprintf('%s\nBuchi automaton has %d states\nTime spent to create Buchi: %g secs',...
    message,length(B.S),tiempo);
message_c = sprintf('%sTime of generating the Buchi automaton: %g secs\n',message_c,tiempo);
time_c = time_c + tiempo;
data.B=B;
set(gcf,'UserData',data);

if (data.optim.paramWith.interM > 2 * length(data.B.S))
    choiceMenu = questdlg(sprintf('The number of intermedate markings is greater than twice the number of states in Buchi automaton (%d states). Do you want to limit the number of intermediate markings to %d (for prefix and suffix)?',...
        length(data.B.S),2*length(data.B.S)),'Robot Motion Toolbox - Path planning with PN models and Buchi included','Yes','No','Yes');
    if strcmpi(choiceMenu,'Yes')
        data.optim.paramWith.interM = 2 * length(data.B.S);
    end
end

% use new function to reduce transitions in Quontient Buchi PN
tic;
[Pre,Post,PreV, PostV, idxV, m0,final_places] = rmt_construct_PN_ltl(Pre,Post,m0,data.Tr.props, data.B);%final places - places corresponding to the final states in the Buchi automaton

tiempo = toc;
message = sprintf('%s\nPetri net system including Buchi and observations has %d places and %d transitions\nTime spent for creating it: %g secs',...
    message,size(Pre,1),size(Pre,2),tiempo);
message_c = sprintf('%sTime of generating the quotient PN model with Buchi included: %g secs\n',message_c,tiempo);
time_c = time_c + tiempo;
data.Pre_full = Pre;
data.Post_full = Post;
set(gcf,'UserData',data);%to save data

vect_fmin = []; % this vector stores all the cost function values for all final states
cell_X = {}; % store solution X for all final states

for idx_fs = 1:length(final_places)

% [A,b,Aeq,beq,cost] = rmt_construct_constraints_ltl(Pre,Post,m0, nplaces_orig, ntrans_orig,...
%     length(data.Tr.props) , 2*data.optim.paramWith.interM, final_places);

% compute prefix for final state idx_fs
tic;
[A,b,Aeq,beq,cost] = rmt_construct_constraints_ltl_prefix(PreV,PostV,m0, ntrans_orig, ...
     data.optim.paramWith.interM, final_places(idx_fs));
tiempo = toc;

message = sprintf('%s\n*****************************************************************',message);
message = sprintf('%s\n\nFinal state %g\n',message, B.F(idx_fs));

message_c = sprintf('%s-----------------------------------------------------------\n',message_c);
message_c = sprintf('%s\nFinal state: %g\n',message_c,B.F(idx_fs));
message_c = sprintf('%sTime of creating the optimization problem on quotient PN: %g secs\n',message_c,tiempo);
time_c = time_c + tiempo;

data = get(gcf,'UserData');
% Part about analysis with Buchi Automaton
%if strcmpi(get(data.optim.menuCplex,'Checked'),'on')
%%%%%%%%%
ctype='';
for i = 1 : size(Aeq,1)
    ctype = sprintf('%sS',ctype);
end
for i = 1 : size(A,1)
    ctype = sprintf('%sU',ctype);
end
vartype = '';
for i = 1 : size(Aeq,2)/(size(PreV,1)+size(PreV,2))
    for j = 1 : size(PreV,1)
        vartype = sprintf('%sC',vartype); %put the markings as real
    end
    for j = 1 : size(PreV,2)
        vartype = sprintf('%sI',vartype); %put the sigma as integer
    end
end

message = sprintf('%s\nTotal number of variables in the MILP problem (quotient PN): %d for %d intermediate markings',...
    message,size(Aeq,2),data.optim.paramWith.interM);
message = sprintf('%s\nThe optimization problem has %d equality contraints and %d inequality constraints (quotient PN).',...
    message, size(Aeq,1), size(A,1));

tic;
switch solver
    case 'cplex'
        [xmin_pref,f,~] = cplexmilp(cost,A,b,Aeq,beq,[],[],[],zeros(1,size(A,2)),[],vartype);
    case 'glpk'
        [xmin_pref,f,~] = glpk(cost,[Aeq; A],[beq; b],zeros(1,size(A,2)),[],ctype,vartype);
    case 'intlinprog'
        [xmin_pref,f,~] = intlinprog(cost, 1:length(cost), A, b, Aeq, beq, zeros(1,length(cost)), []);
end
time = toc;
if isempty(f)%no solution
    uiwait(errordlg('Error solving the ILP on quotient PN. The problem may have no feasible solution. Increase k(Setup -> Parameters for MILP PN with Buchi)!',...
        'Robot Motion Toolbox','modal'));
    return;
end

message = sprintf('%s\n\n---------------------- PREFIX -------------------',message);
% message = sprintf('%s\nFunction value for first MILP - prefix: %g \n', message, f);
message = sprintf('%s\nTime of solving the MILP - prefix (trajectory on quotient PN): %g secs\n', message, time);
total_time = total_time + time;
total_time_MILP = total_time_MILP + time;
message_c = sprintf('%sTime of finding a path in the quotient PN with Buchi - prefix: %g secs\n',message_c,time);
time_c = time_c + time;
message = sprintf('%s\n=======================================================',message);
message = sprintf('%s\nInitial solution on the reduced Petri net system',message);
message = sprintf('%s\n=======================================================\n',message);

% After the optimization problem was solved, an
% initial solution was obtained on the reduced system

% check the active observations after prefix 
[active_observations, possible_regions, number_of_robots, marking_new, message] = rmt_check_active_observations(xmin_pref,PreV,PostV,m0,data,nplaces_orig,ntrans_orig,message);

% check if the last active observations are a subset of observations in the
% self-loop of the final state
flag_act_obs = 0; 
temp_fs = B.F(idx_fs);
for idx_obs = 1:size(B.new_trans{temp_fs,temp_fs},1)
    if length(find(intersect(active_observations{end},B.new_trans{temp_fs,temp_fs}(idx_obs,:)) == active_observations{end})) == length(active_observations{end}) | ...
            B.new_trans{temp_fs,temp_fs} == Inf
        flag_act_obs = 1; % final state has self-loop on True or the active observations are included in the self-loop
    end
end

% update initial marking for MILP 1.2
No_obstacles = data.Nobstacles;

m0_fs = m0;
m0_fs(1:length(data.Tr.Cells)) = marking_new;

idx_obs = data.Tr.obs(find(marking_new));
temp_m0_fs = m0_fs(length(data.Tr.Cells)+1:length(data.Tr.Cells) + No_obstacles);
idx_obs_act = idx_obs(find(idx_obs <= No_obstacles));
temp_m0_fs(idx_obs_act) = 1;
m0_fs(length(data.Tr.Cells)+1:length(data.Tr.Cells) + No_obstacles) = temp_m0_fs;

temp_m0_fs = m0_fs(2*length(data.Tr.Cells):2*length(data.Tr.Cells) + No_obstacles-1);
temp_m0_fs(idx_obs_act) = 0;
m0_fs(2*length(data.Tr.Cells):2*length(data.Tr.Cells) + No_obstacles-1) = temp_m0_fs;

m0_fs(final_places(idx_fs)) = 1;
m0_fs(end - length(B.S) + 1) = 0;

% if flag_act_obs == 0, solve MILP 1.2 for suffix

if flag_act_obs == 0 
[A,b,Aeq,beq,cost] = rmt_construct_constraints_ltl_suffix(PreV,PostV,m0_fs, ntrans_orig,...
     data.optim.paramWith.interM, final_places(idx_fs));

data = get(gcf,'UserData');
% Part about analysis with Buchi Automaton
%if strcmpi(get(data.optim.menuCplex,'Checked'),'on')
%%%%%%%%%
ctype='';
for i = 1 : size(Aeq,1)
    ctype = sprintf('%sS',ctype);
end
for i = 1 : size(A,1)
    ctype = sprintf('%sU',ctype);
end
vartype = '';
for i = 1 : size(Aeq,2)/(size(PreV,1)+size(PreV,2))
    for j = 1 : size(PreV,1)
        vartype = sprintf('%sC',vartype); %put the markings as real
    end
    for j = 1 : size(PreV,2)
        vartype = sprintf('%sI',vartype); %put the sigma as integer
    end
end

message = sprintf('%s\nTotal number of variables in the MILP problem (quotient PN): %d for %d intermediate markings',...
    message,size(Aeq,2),data.optim.paramWith.interM);
message = sprintf('%s\nThe optimization problem has %d equality contraints and %d inequality constraints (quotient PN).',...
    message, size(Aeq,1), size(A,1));

tic;
switch solver
    case 'cplex'
        [xmin_suff,f_suff,~] = cplexmilp(cost,A,b,Aeq,beq,[],[],[],zeros(1,size(A,2)),[],vartype);
    case 'glpk'
        [xmin_suff,f_suff,~] = glpk(cost,[Aeq; A],[beq; b],zeros(1,size(A,2)),[],ctype,vartype);
    case 'intlinprog'
        [xmin_suff,f_suff,~] = intlinprog(cost, 1:length(cost), A, b, Aeq, beq, zeros(1,length(cost)), []);
end
time = toc;
total_time_MILP = total_time_MILP + time;
if isempty(f_suff)%no solution
    uiwait(errordlg('Error solving the ILP on quotient PN. The problem may have no feasible solution. Increase k(Setup -> Parameters for MILP PN with Buchi)!',...
        'Robot Motion Toolbox','modal'));
    return;
end 
message = sprintf('%s\n\n---------------------- SUFFIX --------------------',message);
% message = sprintf('%s\nFunction value for first MILP - suffix: %g \n', message, f_suff);
message = sprintf('%s\nTime of solving the MILP - suffix (trajectory on quotient PN): %g secs\n', message, time);
total_time = total_time + time;
message_c = sprintf('%sTime of finding a path in the quotient PN with Buchi - suffix: %g secs\n',message_c,time);
time_c = time_c + time;
message = sprintf('%s\n=======================================================',message);
message = sprintf('%s\nInitial solution on the reduced Petri net system',message);
message = sprintf('%s\n=======================================================\n',message);

% check the active observations after suffix 
[active_observations_suff, possible_regions_suff, number_of_robots_suff, marking_new, message] = rmt_check_active_observations(xmin_suff,PreV,PostV,m0_fs,data,nplaces_orig,ntrans_orig,message);

possible_regions = [possible_regions possible_regions_suff];
active_observations = [active_observations active_observations_suff];
number_of_robots = [number_of_robots number_of_robots_suff];

%remove eventually identical observations
for j = length(active_observations):-1:2
    if (isempty(setxor(active_observations{j},active_observations{j-1})) && ...
            isempty(setxor(unique([possible_regions{j}{:}]),unique([possible_regions{j-1}{:}]))))
        active_observations(j) = [];
        possible_regions(j) = [];
        number_of_robots(j) = [];
    end
end

end

% print messages
message = sprintf('%s\n\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0>eps*10^5)),mat2str(m0(m0>eps*10^5)));

message = sprintf('%s\n=======================================================',message);
message = sprintf('%s\nProject the solution to the initial transition system with CPLEX',message);
message = sprintf('%s\n=======================================================\n',message);

% second MILP - project the solution

[Pre,Post] = rmt_construct_PN(data.T.adj);
m0_ps = data.T.m0;
nplaces = size(Post,1);
ntrans = size(Post,2);
steps = 1;
message = sprintf('%s\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0_ps>eps*10^5)),mat2str(m0_ps(m0_ps>eps*10^5)));

tic;

[A,b,Aeq,beq,cost] = rmt_construct_constraints_ltl_project_sol(Pre,Post,m0_ps, number_of_robots,...
    possible_regions, nplaces, ntrans, data);

time = toc;
message = sprintf('%s\n\nTotal time to construct the second MILP (project the solution): %g secs', message,time);

ctype='';
for i = 1 : size(Aeq,1)
    ctype = sprintf('%sS',ctype);
end
for i = 1 : size(A,1)
    ctype = sprintf('%sU',ctype);
end
vartype = '';
for i = 1 : size(Aeq,2)/(nplaces+ntrans)
    for j = 1 : size(Pre,1)
        vartype = sprintf('%sC',vartype); %put the markings as real
    end
    for j = 1 : size(Pre,2)
        vartype = sprintf('%sI',vartype); %put the sigma as integer
    end
end


tic;
switch solver
    case 'cplex'
        [X,f,~] = cplexlp(cost,A,b,Aeq,beq,zeros(1,size(Aeq,2)),[]);
    case 'glpk'
        [X,f,~] = glpk(cost,[Aeq; A],[beq; b],zeros(1,size(A,2)),[],ctype,vartype);
    case 'intlinprog'
        [X,f,~] = intlinprog(cost, 1:length(cost), A, b, Aeq, beq, zeros(1,length(cost)), []);
        
end
time = toc;

message = sprintf('%s\n\n --------------------\n\n',message);
message = sprintf('%s\nFunction value second MILP: %g \n', message, f);
message = sprintf('%s\nTotal number of variables in the LP problem (project the solution): %d',...
    message,size(Aeq,2));
message = sprintf('%s\nThe LP has %d equality contraints and %d inequality constraints (project the solution).',...
    message, size(Aeq,1), size(A,1));


total_time_MILP = total_time_MILP + time;
message = sprintf('%s\nTotal time for solving LPP to project the solution: %g secs', message,time);
total_time = total_time + time;
message_c = sprintf('%sTime for projecting the solution on the full model: %g secs\n',message_c,time);
message_c = sprintf('%sTotal time for MILPs: %g secs\n',message_c,total_time_MILP);
time_c = time_c + time;

if isempty(f)
    uiwait(errordlg('Error solving LPP to project the solution!','Robot Motion Toolbox','modal'));
    return;
end

% save all 
vect_fmin = [vect_fmin f];
cell_X{idx_fs} = X;
end

% memorize final state with the minimum cost function value
[temp_fmin, idx_fmin] = min(vect_fmin);

message = sprintf('%s\n\n______________________________________________________\n\n Selected final state: %g\n Cost function value: %g \n', message,B.F(idx_fmin), temp_fmin);
message = sprintf('%s\n\nTotal time for MILPs: %g\n', message,total_time_MILP);

Run_cells = data.RO';
[message,Run_cells] = rmt_path_planning_ltl_with_buchi_trajectories(cell_X{idx_fmin},Pre,Post,Run_cells,Run_cells,message);

%%Execution monitoring strategy starts from here:
rob_traj = rmt_rob_cont_traj_new(data.T,Run_cells,data.initial);    %continuous trajectory of each robot
data.trajectory = rob_traj;

cla(data.handle_env);
rmt_represent_atomic_props(data.T.Vert,data.propositions);    %represent all atomic props and store handles

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
    plot(rob_traj{r}(1,1),rob_traj{r}(2,1),data.rob_plot.line_color{r},...
        'Marker',data.rob_plot.marker{r},'LineWidth',data.rob_plot.line_width{r});
    plot(rob_traj{r}(1,:),rob_traj{r}(2,:),data.rob_plot.line_color{r},...
        'LineWidth',data.rob_plot.line_width{r});
    plot(rob_traj{r}(1,end),rob_traj{r}(2,end),data.rob_plot.line_color{r},...
        'Marker',data.rob_plot.marker{r},'LineWidth',data.rob_plot.line_width{r},'Color','r');
end

% compute synchronizations
Synch = [];
for i = 1 : size(Run_cells,1)
    traj = Run_cells(i,:);
    for j = 2 : length(traj)
        if ( data.T.obs(traj(j-1)) ~= data.T.obs(traj(j)) )
            Synch = [Synch j];
        end
    end
end

% if only one robot is changing the observation not necessary to
% synchronize. chose from set Synch only duplicated elements
[~, ind] = unique(Synch);
% duplicate indices
duplicate_ind = setdiff(1:length(Synch), ind);
% duplicate values
Synch = unique(Synch(duplicate_ind));
disp(Synch);

rob_color={'g','c','m','k','y','r','b'};
data.rob_plot.line_color = rob_color;

for r=1:length(rob_traj)    %plot trajectories of robots
    for tt = 1 : length(Synch)
        plot(rob_traj{r}(1,Synch(tt)),rob_traj{r}(2,Synch(tt)),data.rob_plot.line_color{r},...
            'Marker',data.rob_plot.marker{r},'LineWidth',data.rob_plot.line_width{r},'Color','b');
    end
end

if ~isempty(Synch)
    message = sprintf('%s\n\nSynchonization points: ',message);
    for i = 1 : length(Synch)
        message = sprintf('%s %s',message,mat2str(Run_cells(:,Synch(i))));
    end
end

message_c = sprintf('%sTotal time: %g secs\n',message_c,time_c);
disp(message_c);

message2 = '';
button = questdlg('Save the details to a text file?','Robot Motion Toolbox');
if strcmpi(button,'Yes')
    [filename, pathname] = uiputfile('*.txt', 'Save experiments as');
    fileID = fopen(fullfile(pathname, filename),'w');
    fprintf(fileID,'%s\n\nTotal time to solve both optimization problems: %d',message,total_time);
    fclose(fileID);
end

set(gcf,'UserData',data);%to save data
