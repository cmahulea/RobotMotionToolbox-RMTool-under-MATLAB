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

function rmt_path_planning_boolean
%Path-planning with boolean specificaion and Petri net models (mathematical programming approach)
%%approach without collision avoidance (it is only soft constraint) TAC 2018

data = get(gcf,'UserData');
%Takes the string containing the Boolean formula inserted
data.Bool_formula = get(findobj(gcf,'Tag','booleanformula'),'String');
button = questdlg('Solve on the full trasition system or on the reduced one?',...
    'Robot Motion Toolbox','Full system','Reduced system','Full system');
if strcmpi(button,'Reduced system')
    reduced = 1;
    tic;
    if ~isfield(data,'Tr')
        data.Tr = rmt_quotient_T_new(data.T); %quotient of partition T, with fewer states (based on collapsing states with same observables in same connected component with same obs)
    end
    
    % Construction of the Petri net
    [Pre,Post] = rmt_construct_PN(data.Tr.adj);
    m0=data.Tr.m0;
    message = sprintf('Petri net system has %d places and %d transitions\nTime spent for creating it: %g secs', size(Pre,1),size(Pre,2),toc);
    uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
    tic;
    [A,b,Aeq,beq,cost] = rmt_construct_constraints(Pre,Post,m0,data.Tr.props,data.optim.param_boolean.kappa,data.Bool_formula);
    message2 = sprintf('\nMathematical program has %d variables and %d equality constraints and %d inequality constraints;\nTime spent for creating the problem: %g secs\n', size(A,2), size(Aeq,1), size(A,1), toc);
    message = sprintf('%s%s', message, message2);
    uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));
else
    reduced = 0;
    [Pre,Post] = rmt_construct_PN(data.T.adj);
    m0=data.T.m0;
    tic;
    [A,b,Aeq,beq,cost] = rmt_construct_constraints(Pre,Post,m0,data.T.props,data.optim.param_boolean.kappa,data.Bool_formula);
    time = toc;
    message = sprintf('Petri net system has %d places and %d transitions\nTime spent for creating it: %g secs', size(Pre,1),size(Pre,2),time);
    message = sprintf('%s\nThe MILP has %d variables and %d equality constraints and %d inequality constraints;\nTime spent for creating the problem: %g secs\nNumber of intermediate markings: %d\n', ...
        message, size(A,2), size(Aeq,1), size(A,1),time, data.optim.param_boolean.kappa);
    uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
end
data.Pre = Pre;
data.Post = Post;
set(gcf,'UserData',data);%to save data

data = get(gcf,'UserData');
if strcmp(get(data.optim.menuCplex,'Checked'),'on')
    message = sprintf('%s\n\nThe MILP solution is with CPLEX\n\n', message);
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    ctype='';
    for i = 1 : size(Aeq,1)
        ctype = sprintf('%sS',ctype);
    end
    for i = 1 : size(A,1)
        ctype = sprintf('%sU',ctype);
    end
    vartype = '';
    for i = 1 : data.optim.param_boolean.kappa
        for j = 1 : size(Pre,1)
            vartype = sprintf('%sC',vartype); %put the markings as real
        end
        for j = 1 : size(Pre,2)
            vartype = sprintf('%sI',vartype); %put the sigma as integer
        end
    end
    for i = 1 : 2 * length(data.T.props)
        vartype = sprintf('%sB',vartype); %put the binary variable
    end
    vartype = sprintf('%sC',vartype); %put the infinite norm b as real
    tic;
    [xmin,f,~] = cplexmilp(cost,A,b,Aeq,beq,[],[],[],zeros(1,size(A,2)),[],vartype);
    bb = xmin(end);
else
    % Solution with GLPK
    message = sprintf('%s\n\nThe MILP solution is with GLPK\n\n', message);
    ctype1='';
    ctype2='';
    for i = 1 : size(Aeq,1)
        ctype2 = sprintf('%sS',ctype2);
    end
    for i = 1 : size(A,1)
        ctype1 = sprintf('%sU',ctype1);
    end
    
    vartype = '';
    for i = 1 : data.optim.param_boolean.kappa
        for j = 1 : size(Pre,1)
            vartype = sprintf('%sC',vartype); %put the markings as real
        end
        for j = 1 : size(Pre,2)
            vartype = sprintf('%sI',vartype); %put the sigma as integer
        end
    end
    for i = 1 : 2 * length(data.T.props)
        vartype = sprintf('%sB',vartype); %put the binary variable
    end
    vartype = sprintf('%sC',vartype); %put the infinite norm b as real
    tic;
    Atot = [Aeq; A];
    btot= [beq; b];
    ctype = [ctype2 ctype1];
%    [xmin,f,~] = glpk(cost,Atot,btot,zeros(1,size(A,2)),[],ctype,vartype);
    [xmin, fmin, status, extra] = glpk(cost,Atot,btot,zeros(1,size(A,2)),[],ctype,vartype);
    bb = xmin(end);
end
time = toc;
if (isempty(xmin)) %no solution
    uiwait(errordlg('Error solving the ILP. The problem may have no feasible solution or k should be increased!','Robot Motion Toolbox','modal'));
    return;
end
message2 = sprintf('\nTime of solving the MILP: %g secs\n', time);
message = sprintf('%s%s', message, message2);
uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));

if (reduced == 1)
    message = sprintf('%s\n=======================================================',message);
    message = sprintf('%s\nInitial solution on the reduced transition system',message);
    message = sprintf('%s\n=======================================================\n',message);
end

message = sprintf('%s\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0>eps*10^5)),mat2str(m0(m0>eps*10^5)));
if (reduced == 0)
    Run_cells = data.RO'; %rmt_marking2places(m0);
else
    original_places{1} = [];
    temp = rmt_marking2places(m0);
    for j = 1 : length(temp)
        original_places{1} = union(original_places{1},data.Tr.Cells{temp(j)});
    end
end
nplaces = size(Pre,1);
ntrans = size(Pre,2);
step = 1;
for i = 1 : data.optim.param_boolean.kappa
    m = xmin((i-1)*(nplaces+ntrans)+1 : (i-1)*(nplaces+ntrans)+nplaces);
    fire = xmin((i-1)*(nplaces+ntrans)+nplaces+1 : (i-1)*(nplaces+ntrans)+nplaces+ntrans);
    if (max(fire) > 0)
        message = sprintf('%s\n============STEP %d =============\n',message,step);
        step = step + 1;
        message=sprintf('%s\nMarking [ %s ] = %s\n',message,mat2str(find(m>eps*10^5)),mat2str(m(m>eps*10^5)));
        message = sprintf('%s\nSigma [ %s ] = %s\n',message,mat2str(find(fire>eps*10^5)),mat2str(fire(fire>eps*10^5)));
        if (reduced == 0)
            Run_cells = [Run_cells, rmt_find_next_state(m,m0,Run_cells(:,end),fire,Post,Pre)];  %visited cells (rows)
        else
            temp = rmt_marking2places(m);
            original_places{i+1} = [];
            for j = 1 : length(temp)
                original_places{i+1} = union(original_places{i+1},data.Tr.Cells{temp(j)});
            end
        end
    else
        if (reduced == 1)
            original_places{i+1} = original_places{i};
        end
    end
    m0 = m;
end
message = sprintf('%sBoolean variables in solutions are: %s\n',...
    message,mat2str(xmin(data.optim.param_boolean.kappa*(size(Pre,1)+size(Pre,2))+1:end)));

if (reduced == 1)
    %                     options = cplexoptimset('cplex');
    %                     options.display='off';
    %                     options.lpmethod=1;
    message = sprintf('%s\n================================================================',message);
    message = sprintf('%s\nProject the solution to the initial transition system with GLPK',message);
    message = sprintf('%s\n================================================================\n',message);
    [Pre,Post] = rmt_construct_PN(data.T.adj);
    m0 = data.T.m0;
    steps = 1;
    message = sprintf('%s\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0>eps*10^5)),mat2str(m0(m0>eps*10^5)));
    Run_cells = data.RO';%rmt_marking2places(m0);
    tic;
    for i = 1 : data.optim.param_boolean.kappa
        m = xmin((i-1)*(nplaces+ntrans)+1 : (i-1)*(nplaces+ntrans)+nplaces);
        fire = xmin((i-1)*(nplaces+ntrans)+nplaces+1 : (i-1)*(nplaces+ntrans)+nplaces+ntrans);
        if (max(fire) > 0)
            % places = find(m>eps*10^5);%final marked places
            places = rmt_marking2places(m);%final marked places
            markings = m(m>eps*10^5); %number of tokens in final marked places
            temp = setdiff(1:size(Post,1),union(original_places{i},original_places{i+1}));
            TT = data.T;
            for j = 1 : length(temp)
                TT.adj(temp(j),:)=0;
                TT.adj(:,temp(j))=0;
            end
            [Pre_new,Post_new] = rmt_construct_PN(TT.adj);
            Aeq = [eye(size(Pre_new,1)) -Post_new+Pre_new]; %state equation: m = m0 + C \sigma
            beq = m0;
            %add constraints on the intermediate final marking
            % for j = 1 : length(places)
            for j = 1 : length(markings)
                m_final = zeros(1,size(Pre_new,1));
                m_final(data.Tr.Cells{places(j)}) = 1;
                Aeq = [Aeq; m_final zeros(1,size(Pre_new,2))];
                beq = [beq; markings(j)];
            end
            %[X,~,exitflag] = cplexlp(ones(1,size(Pre_new,2)+size(Pre_new,1)),[],[],...
            %          Aeq,beq,zeros(1,size(Pre_new,1)+size(Pre_new,2)),[],[],options);
            ctype3='';
            for i = 1 : size(Aeq,1)
                ctype3 = sprintf('%sS',ctype3);
            end
            vartype3 = '';
            for j = 1 : size(Aeq,2)
                vartype3 = sprintf('%sC',vartype3); %put the sigma as integer
            end
            [X,~,f] = glpk(ones(1,size(Pre_new,2)+size(Pre_new,1)),Aeq,beq,zeros(1,size(Pre_new,1)+size(Pre_new,2)),[],ctype3,vartype3,1);
            if (f~=5) %glpk staus=5 is optimal solution found
                uiwait(errordlg('Error solving LPP to project the solution!','Robot Motion Toolbox','modal'));
                return;
            end
            
            [ret,Run_cells] = rmt_find_sequence(m0, round(X(size(Pre_new,1)+1:end)), Pre_new, Post_new,steps,Run_cells);
            steps = steps + sum(round(X(size(Pre,1)+1:end)))+1;
            fire = round(X(size(Pre_new,1)+1:end));
            disp(ret);
            m0 = round(X(1:size(Pre_new,1))); % final marking is the initial marking for the next step
        end
    end
    time = toc;
    message2 = sprintf('Total time for solving %d LPPs for projecting the solution: %g secs', ...
        data.optim.param_boolean.kappa, time);
    uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));
    message = sprintf('%s\n%s',message,message2);
end
cla(data.handle_env);
%%Execution monitoring strategy starts from here:
N_r = length(data.RO);
current_pos=cell(1,N_r);
for r=1:N_r
    current_pos{r}=data.initial{r};%R_trajs{r}(:,1); %current_pos - initial (current) position of robots (cell, current_pos{i} is a column vector with 2 rows)
end
rmt_represent_atomic_props(data.T.Vert,data.propositions);    %represent all atomic props and store handles

rob_traj = rmt_rob_cont_traj_new(data.T,Run_cells,data.initial);    %continuous trajectory of each robot
data.trajectory = rob_traj;

message = sprintf('%s\nSOLUTION - runs of robots: \nInfinite norm in solution (b):%s',message,num2str(bb));
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
        'Marker',data.rob_plot.marker{r},'LineWidth',data.rob_plot.line_width{r});
end
button = questdlg('Save the details to a text file?','Robot Motion Toolbox');
if strcmpi(button,'Yes')
    [filename, pathname] = uiputfile('*.txt', 'Save experiments as');
    fileID = fopen(fullfile(pathname, filename),'w');
    fprintf(fileID,'%s',message);
    fclose(fileID);
end

set(gcf,'UserData',data);%to save data
