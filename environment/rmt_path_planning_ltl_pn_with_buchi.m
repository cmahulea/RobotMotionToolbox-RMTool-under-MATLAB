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

%create the observation set
N_r = length(data.RO); %In RO there is a region that contains a token (robot)
N_p = data.Nobstacles;%number of regions of interest
temp_obs = rmt_observation_set_new(data.T.OBS_set,N_p,N_r);

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
tic;
[Pre,Post,m0,final_places] = rmt_construct_PN_ltl(Pre,Post,m0,data.Tr.props, data.B,temp_obs);%final places - places corresponding to the final states in the Buchi automaton
tiempo = toc;
message = sprintf('%s\nPetri net system including Buchi and observations has %d places and %d transitions\nTime spent for creating it: %g secs',...
    message,size(Pre,1),size(Pre,2),tiempo);
message_c = sprintf('%sTime of generating the quotient PN model with Buchi included: %g secs\n',message_c,tiempo);
time_c = time_c + tiempo;
data.Pre_full = Pre;
data.Post_full = Post;
set(gcf,'UserData',data);%to save data

tic;
[A,b,Aeq,beq,cost] = rmt_construct_constraints_ltl(Pre,Post,m0, nplaces_orig, ntrans_orig,...
    length(data.Tr.props) , 2*data.optim.paramWith.interM, final_places);
tiempo = toc;
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
for i = 1 : size(Aeq,2)/(size(Pre,1)+size(Pre,2))
    for j = 1 : size(Pre,1)
        vartype = sprintf('%sC',vartype); %put the markings as real
    end
    for j = 1 : size(Pre,2)
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
        [xmin,f,~] = cplexmilp(cost,A,b,Aeq,beq,[],[],[],zeros(1,size(A,2)),[],vartype);
    case 'glpk'
        [xmin,f,~] = glpk(cost,[Aeq; A],[beq; b],zeros(1,size(A,2)),[],ctype,vartype);
end
time = toc;
if isempty(f)%no solution
    uiwait(errordlg('Error solving the ILP on quotient PN. The problem may have no feasible solution. Increase k(Setup -> Parameters for MILP PN with Buchi)!',...
        'Robot Motion Toolbox','modal'));
    return;
end
message = sprintf('%s\nTime of solving the MILP (trajectory on quotient PN): %g secs\n', message, time);
total_time = total_time + time;
message_c = sprintf('%sTime of finding a path in the quotient PN with Buchi: %g secs\n',message_c,time);
time_c = time_c + time;
message = sprintf('%s\n=======================================================',message);
message = sprintf('%s\nInitial solution on the reduced Petri net system',message);
message = sprintf('%s\n=======================================================\n',message);

% After the optimization problem was solved, an
% initial solution was obtained on the reduced system
m0_old = m0;
m0 = m0(1:nplaces_orig);
m0_Buchi = m0_old(nplaces_orig+2*length(data.Tr.props)+1:end);
m0_obs = m0_old(nplaces_orig+1:nplaces_orig+length(data.Tr.props));
message = sprintf('%s\n\n\t Initial state of Buchi = %s',message,mat2str(find(m0_Buchi)));
active_observations{1} = find(m0_obs);
if isempty(active_observations{1})
    message = sprintf('%s\n\t No active observations at initial state',message);
else
    message = sprintf('%s\n\t Active observations = %s',message,mat2str(active_observations{1}));
end
pos_regions={};
temp = find(m0);
marking_temp = zeros(length(temp));
for i = 1 : length(temp)
    pos_regions{i} = data.Tr.Cells{temp(i)};
    marking_temp(i) = m0(temp(i));
end
possible_regions{1} = pos_regions;
number_of_robots{1} = marking_temp;
for i = 1 : 2*data.optim.paramWith.interM
    if (i/2 == round(i/2))
        trans_buchi=find([xmin((i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+ntrans_orig+1:i*(size(Pre,1)+size(Pre,2)))]);
        input_place = find(Pre(:,trans_buchi+ntrans_orig)) ;
        input_place = input_place(input_place>nplaces_orig+2*length(data.Tr.props))-nplaces_orig-2*length(data.Tr.props); %take only the place of the buchi
        output_place = find(Post(:,trans_buchi+ntrans_orig));
        output_place = output_place(output_place>nplaces_orig+2*length(data.Tr.props))-nplaces_orig-2*length(data.Tr.props);
        message = sprintf('%s\n Transition in Buchi from state %d to state %d with observation (%s)',message,...
            input_place,output_place,mat2str(find([xmin((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+1:(i-1)*(size(Pre,1)+size(Pre,2))+length(data.Tr.props)+nplaces_orig)])));
        message = sprintf('%s\n\t State of Buchi in step %d = %s',message,i/2,...
            mat2str(find([xmin((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+2*length(data.Tr.props)+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1))])));
        
        active_observations{length(active_observations)+1} = find([xmin((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+1:(i-1)*(size(Pre,1)+size(Pre,2))+length(data.Tr.props)+nplaces_orig)]);
        if isempty(active_observations{length(active_observations)})
            message = sprintf('%s\n\t No active observations at step %d',message,i/2);
        else
            message = sprintf('%s\n\t Active observations at step %d = %s',message,i/2,mat2str(active_observations{length(active_observations)}));
        end
        %take the new marking of the robot model
        marking_new = [xmin((i-1)*(size(Pre,1)+size(Pre,2))+1:(i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig)];
        temp = find(marking_new);%marking of places modeling the team
        pos_regions={};
        marking_temp = zeros(1,length(temp));
        for j = 1 : length(temp)
            pos_regions{j} = data.Tr.Cells{temp(j)};
            marking_temp(j) = marking_new(temp(j));
        end
        
        possible_regions{length(possible_regions)+1} = pos_regions;
        number_of_robots{length(number_of_robots)+1} = marking_temp; %number of robots in each macro region
        message = sprintf('%s\n\t Possible regions for the robots',message);
        for k = 1 : length(pos_regions)
            message = sprintf('%s\n\t\t --- %s',message,mat2str(pos_regions{k}));
        end
    end
end
%remove eventually identical observations
for j = length(active_observations):-1:2
    if (isempty(setxor(active_observations{j},active_observations{j-1})) && ...
            isempty(setxor(unique([possible_regions{j}{:}]),unique([possible_regions{j-1}{:}]))))
        active_observations(j) = [];
        possible_regions(j) = [];
        number_of_robots(j) = [];
    end
end
message = sprintf('%s\n\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0>eps*10^5)),mat2str(m0(m0>eps*10^5)));

message = sprintf('%s\n=======================================================',message);
message = sprintf('%s\nProject the solution to the initial transition system with CPLEX',message);
message = sprintf('%s\n=======================================================\n',message);
[Pre,Post] = rmt_construct_PN(data.T.adj);
m0 = data.T.m0;
nplaces = size(Post,1);
ntrans = size(Post,2);
steps = 1;
message = sprintf('%s\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0>eps*10^5)),mat2str(m0(m0>eps*10^5)));
Run_cells = data.RO';%rmt_marking2places(m0);

tic;
Aeq=[];
beq = [];
A=[];
b = [];
%variables: m1 sigma1 m2 sigma2 ....
Aeq = [eye(nplaces) -(Post-Pre)]; %state equation
beq = m0;

%in the first step, at m1 the same numbers of robots remains in he
%regions with the same observation as at m0
for i = 1 : length(possible_regions{1})
    temp = zeros(1,nplaces);
    temp(possible_regions{1}{i})=1;
    Aeq = [Aeq; temp zeros(1,ntrans)];
    beq = [beq; temp*m0];
end
other_regions = setdiff(data.T.Q,unique([possible_regions{1}{:}]));
not_fire = zeros(1,ntrans);
for k = 1 : length(other_regions)
    not_fire(find(Post(other_regions(k),:))) = 1;
end
Aeq = [Aeq; zeros(1,nplaces) not_fire];
beq = [beq; 0];

A = [zeros(nplaces,nplaces) Post]; %constraint for collision avoidance
b = [ones(nplaces,1)-m0];

%N_r - number of robots in order to obtain collision free trajectories
for j = 2 : N_r
    Aeq = [Aeq zeros(size(Aeq,1),nplaces+ntrans)];
    A = [A zeros(size(A,1),nplaces+ntrans)]; %add nplaces+ntrans columns corresponding to the new intermediate marking
    Aeq = [Aeq; zeros(nplaces,(j-2)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) eye(nplaces) -(Post-Pre)]; %state equation
    beq = [beq; zeros(nplaces,1)];
    %in the first step, at m1 the same numbers of robots remains in the
    %regions with the same observation as at m0
    for i = 1 : length(possible_regions{1})
        temp = zeros(1,nplaces);
        temp(possible_regions{1}{i})=1;
        %keep the same number of robots in the regions with observations
        Aeq = [Aeq; zeros(1,(j-2)*(nplaces+ntrans)) temp zeros(1,ntrans) -temp zeros(1,ntrans)];
        beq = [beq; 0];
    end
    %not fire transitions to change the regions and get other
    %observations
    Aeq = [Aeq; zeros(1,(j-1)*(nplaces+ntrans)) zeros(1,nplaces) not_fire];
    beq = [beq; 0];
    
    %constraint for collision avoidance
    A = [A; zeros(nplaces,(j-2)*(nplaces+ntrans)) eye(nplaces) zeros(nplaces,ntrans) zeros(nplaces,nplaces) Post];
    b = [b; ones(nplaces,1)];
end

% add a new marking in which each robot moves maximum one regions,
% at this marking active the observations
Aeq = [Aeq zeros(size(Aeq,1),nplaces+ntrans)];
A = [A zeros(size(A,1),nplaces+ntrans)]; %add nplaces+ntrans columns corresponding to the new intermediate marking
% fire only one transition
A = [A; zeros(nplaces,(N_r-1)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) zeros(nplaces,nplaces) +Pre];
b = [b;zeros(nplaces,1)];
%state equation
Aeq = [Aeq; zeros(nplaces,(N_r-1)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) eye(nplaces) -(Post-Pre)];
beq = [beq; zeros(nplaces,1)];
%change the regions to the next sets
for j = 1 : length(possible_regions{2})
    temp = zeros(1,nplaces);
    temp(possible_regions{2}{j})=1;
    Aeq = [Aeq; zeros(1,(N_r)*(nplaces+ntrans)) temp zeros(1,ntrans)];
    beq = [beq; number_of_robots{2}(j)];
end

for i = 2 : length(possible_regions)-1
    other_regions = setdiff(data.T.Q,unique([possible_regions{i}{:}]));
    not_fire = zeros(1,ntrans);
    for k = 1 : length(other_regions)
        not_fire(find(Post(other_regions(k),:))) = 1;
    end
    
    for j = 1 : N_r %N_r - number of robots in order to obtain collision free trajectories
        actual_int_mark = size(Aeq,2)/(nplaces+ntrans);
        %add nplaces+ntrans columns corresponding to the new intermediate marking
        Aeq = [Aeq zeros(size(Aeq,1),nplaces+ntrans)];
        A = [A zeros(size(A,1),nplaces+ntrans)];
        %state equation
        Aeq = [Aeq; zeros(nplaces,(actual_int_mark-1)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) eye(nplaces) -(Post-Pre)];
        beq = [beq; zeros(nplaces,1)];
        %keep the same observations
        for k = 1 : length(possible_regions{i})
            temp = zeros(1,nplaces);
            temp(possible_regions{i}{k})=1;
            Aeq = [Aeq; zeros(1,actual_int_mark*(nplaces+ntrans)) temp zeros(1,ntrans)];
            beq = [beq; number_of_robots{i}(k)];
        end
        %do not fire transitions that brings to places with other
        %observations
        Aeq = [Aeq; zeros(1,actual_int_mark*(nplaces+ntrans)) zeros(1,nplaces) not_fire];
        beq = [beq;0];
        %constraint for collision avoidance
        A = [A; zeros(nplaces,(actual_int_mark-1)*(nplaces+ntrans)) eye(nplaces) zeros(nplaces,ntrans) zeros(nplaces,nplaces) Post];
        b = [b; ones(nplaces,1)];
    end
    %add a new marking to perform a transition for Buchi
    %add nplaces+ntrans columns corresponding to the new intermediate marking
    Aeq = [Aeq zeros(size(Aeq,1),nplaces+ntrans)];
    A = [A zeros(size(A,1),nplaces+ntrans)];
    %advance only one transition
    A = [A; zeros(nplaces,(actual_int_mark)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) zeros(nplaces,nplaces) +Pre];
    b = [b;zeros(nplaces,1)];
    %state equation
    Aeq = [Aeq; zeros(nplaces,(actual_int_mark)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) eye(nplaces) -(Post-Pre)];
    beq = [beq; zeros(nplaces,1)];
    %constraints to activate the observation at the final state
    actual_int_mark = size(Aeq,2)/(nplaces+ntrans)-1;
    for k = 1 : length(possible_regions{i+1})
        temp = zeros(1,nplaces);
        temp(possible_regions{i+1}{k})=1;
        Aeq = [Aeq; zeros(1,actual_int_mark*(nplaces+ntrans)) temp zeros(1,ntrans)];
        beq = [beq; number_of_robots{i+1}(k)];
    end
end
cost = [];
for i = 1 : size(Aeq,2)/(nplaces+ntrans)
    cost = [cost zeros(1,nplaces) ones(1,ntrans)];
end

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
end

message = sprintf('%s\nTotal number of variables in the LP problem (project the solution): %d',...
    message,size(Aeq,2));
message = sprintf('%s\nThe LP has %d equality contraints and %d inequality constraints (project the solution).',...
    message, size(Aeq,1), size(A,1));

time = toc;
message = sprintf('%s\nTotal time for solving LPP to project the solution: %g secs', message,time);
total_time = total_time + time;
message_c = sprintf('%sTime for projecting the solution on the full model: %g secs\n',message_c,time);
time_c = time_c + time;

if isempty(f)
    uiwait(errordlg('Error solving LPP to project the solution!','Robot Motion Toolbox','modal'));
    return;
end

Run_cells = data.RO';
[message,Run_cells] = rmt_path_planning_ltl_with_buchi_trajectories(X,Pre,Post,Run_cells,Run_cells,message);

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
