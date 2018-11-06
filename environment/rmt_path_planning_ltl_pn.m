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

function rmt_path_planning_ltl_pn
%Path-planning with LTL specificaion and Petri net models (mathematical programming approach)

disp('START CREATION PETRI NET WITH LTL FORMULA');
data = get(gcf,'UserData');
data.formula = get(findobj(gcf,'Tag','ltlformula'),'String'); %% read LTL formula
tic;
if ~isfield(data,'Tr')
    data.Tr = rmt_quotient_T(data.T); %quotient of partition T, with fewer states (based on collapsing states with same observables in same connected component with same obs)
end

[Pre,Post] = rmt_construct_PN(data.Tr.adj);
m0=data.Tr.m0;
message = sprintf('Petri net system has %d places and %d transitions\nTime spent for creating it: %g secs', size(Pre,1),size(Pre,2),toc);
uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
nplaces_orig = size(Pre,1);
ntrans_orig = size(Pre,2);

%crete the observation set
N_r = length(data.RO); %In RO there is a region that contains a token (robot)
observ_set = data.Tr.OBS_set(1:size(data.Tr.OBS_set,1)-1,:); %Remove free space from initial Obs set
temp_cell=mat2cell( observ_set , ones(1,size(observ_set,1)) , size(observ_set,2) );  %duplicate observables of transition systems
temp_obs=rmt_cartesian_product(temp_cell{:});  %possible observables, on rows (more robots can have the same observables, that's why we use carth product); observables will be labeled with indices of rows (in T.obs)
temp_obs=unique(sort(temp_obs,2),'rows');   %sort rows and remove identical ones (they would have the same observable)
for i=1:size(temp_obs,1)  %modify observables (keep at most one occurence of same prop on each row, and pad with zeros until length
    obs=unique(temp_obs(i,:));    %keep unique elements on each row, and pad wih zeros
    if length(obs)<size(temp_obs,2) %pad with zeros until number of propositions
        obs((end+1):size(temp_obs,2))=0;
    end
    temp_obs(i,:)=obs;
end
temp_obs=unique(sort(temp_obs,2),'rows');   %again remove identical rows (there may appear new repetitions, due to keeping only unique elements on each row and padding with zeros)
temp_obs(end+1,:)=[length(data.Tr.props)+1 , zeros(1,size(temp_obs,2)-1)]; %dummy has index "ind_dummy", and pad with zeros after it

% Creating the automaton Buchi to be included in the global Petri Net
if ~isfield(data,'B')
    % Control on the number of region of interest
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
        
        
        B = rmt_create_buchi(data.formula, temp_obs);
        data.B=B;
    else
        B = rmt_create_buchi(data.formula, temp_obs);
        data.B=B;
    end
    
else
    choice2 = questdlg('Should the Buchi automaton be computed again?', ...
        'Robot Motion Toolbox', ...
        'Yes','No','Yes');
    if strcmpi(choice2,'Yes')
        B = rmt_create_buchi(data.formula, temp_obs);
    else
        B = data.B;
    end
end
data.B=B;
set(gcf,'UserData',data);

tic;
[Pre,Post,m0,final_places] = rmt_construct_PN_ltl(Pre,Post,m0,data.Tr.props, data.B,temp_obs);%final places - places corresponding to the final states in the Buchi automaton
message2 = sprintf('Petri net system including Buchi and observations has %d places and %d transitions\nTime spent for creating it: %g secs', size(Pre,1),size(Pre,2),toc);
message = sprintf('%s\n%s', message, message2);
uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));

data.Pre_full = Pre;
data.Post_full = Post;
set(gcf,'UserData',data);%to save data

% This function return
% Aeq= [I(n_places) - (Post-Pre)] that correspond at the equation mi+1 = mi + (Post-Pre)*sigma
% beq = m0
% A = (m0 - Pre*sigma)
% b = m0
[A,b,Aeq,beq,cost] = rmt_construct_constraints_ltl(Pre,Post,m0, nplaces_orig, ntrans_orig, length(data.Tr.props) , 2*data.intermediateMarkings, final_places);

data = get(gcf,'UserData');
% Part about analysis with Buchi Automaton
if strcmpi(data.cplex_variable,'true')
    %%%%%%%%%
    ctype='';
    for i = 1 : size(Aeq,1)
        ctype = sprintf('%sS',ctype);
    end
    for i = 1 : size(A,1)
        ctype = sprintf('%sU',ctype);
    end
    vartype = '';
    for i = 1 : 2*data.intermediateMarkings
        for j = 1 : size(Pre,1)
            vartype = sprintf('%sC',vartype); %put the markings as real
        end
        for j = 1 : size(Pre,2)
            vartype = sprintf('%sI',vartype); %put the sigma as integer
        end
    end
    
    tic;
    [xmin,f,~] = cplexmilp(cost,A,b,Aeq,beq,[],[],[],zeros(1,size(A,2)),[],vartype);
    time = toc;
    if isempty(f)%no solution
        uiwait(errordlg('Error solving the ILP. The problem may have no feasible solution. Increase k!','Robot Motion Toolbox','modal'));
        return;
    end
    message2 = sprintf('\nTime of solving the MILP: %g secs\n', time);
    message = sprintf('%s%s', message, message2);
    uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));
    
    message = sprintf('%s\n=======================================================',message);
    message = sprintf('%s\nInitial solution on the reduced transition system',message);
    message = sprintf('%s\n=======================================================\n',message);
    
    % After the optimization problem was solved, an
    % initial solution was obtained on the reduced system
    m0_old = m0;
    m0 = m0(1:nplaces_orig);
    m0_Buchi = m0_old(nplaces_orig+length(data.Tr.props)+1:end);
    m0_obs = m0_old(nplaces_orig+1:nplaces_orig+length(data.Tr.props));
    xm = xmin;
    xmin = [];
    ymin = [];
    fprintf(1,'\nInitial state of Buchi = %s',mat2str(find(m0_Buchi)));
    fprintf(1,'\n\tActive observations = %s',mat2str(find(m0_obs)));
    for i = 1 : 2*data.intermediateMarkings
        xmin = [xmin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+1:(i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig)];%marking of places modeling the team
        %fprintf(1,'Marking of places modeling the team at step %d: %s',)
        xmin = [xmin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+ntrans_orig)];%firing vector of places modeling the team
        
        ymin = [ymin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+length(data.Tr.props)+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1))];%marking of places modeling the Buchi
        ymin = [ymin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+ntrans_orig+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+size(Pre,2))];%firing vector of places modeling the buchi
        
        if (i/2 == round(i/2))
            trans_buchi=find([xm((i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+ntrans_orig+1:i*(size(Pre,1)+size(Pre,2)))]);
            input_place = find(Pre(:,trans_buchi+ntrans_orig)) ;
            input_place = input_place(input_place>nplaces_orig+length(data.Tr.props))-nplaces_orig-length(data.Tr.props); %take only the place of the buchi
            output_place = find(Post(:,trans_buchi+ntrans_orig));
            output_place = output_place(output_place>nplaces_orig+length(data.Tr.props))- nplaces_orig-length(data.Tr.props);
            fprintf(1,'\n Transition in Buchi from state %d to state %d with observation (%s)',input_place,output_place,mat2str(find([xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+1:(i-1)*(size(Pre,1)+size(Pre,2))++length(data.Tr.props)+nplaces_orig)])));
            fprintf(1,'\nState of Buchi in step %d = %s',i/2,mat2str(find([xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+length(data.Tr.props)+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1))])));
            %Decomment this part
            fprintf(1,'\n\tActive observations at step %d = %s',i/2,mat2str(find([xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+1:(i-1)*(size(Pre,1)+size(Pre,2))++length(data.Tr.props)+nplaces_orig)])));
        end
        
    end
    fprintf(1,'\n');
    message = sprintf('%s\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0>eps*10^5)),mat2str(m0(m0>eps*10^5)));
    original_places{1} = [];
    temp = rmt_marking2places(m0);
    for j = 1 : length(temp)
        original_places{1} = union(original_places{1},data.Tr.Cells{temp(j)});
    end
    nplaces = nplaces_orig;
    ntrans = ntrans_orig;
    
    for i = 1 : 2*data.intermediateMarkings
        m = xmin((i-1)*(nplaces+ntrans)+1 : (i-1)*(nplaces+ntrans)+nplaces);
        fire = xmin((i-1)*(nplaces+ntrans)+nplaces+1 : (i-1)*(nplaces+ntrans)+nplaces+ntrans);
        if (max(fire) > 0)
            message = sprintf('%s\n============STEP %d =============\n',message,i);
            message=sprintf('%s\nMarking [ %s ] = %s\n',message,mat2str(find(m>eps*10^5)),mat2str(m(m>eps*10^5)));
            message = sprintf('%s\nSigma [ %s ] = %s\n',message,mat2str(find(fire>eps*10^5)),mat2str(fire(fire>eps*10^5)));
            %                 if (reduced == 0)
            %                     Run_cells = [Run_cells, rmt_marking2places(m)];  %visited cells (rows)
            %                 else
            temp = rmt_marking2places(m);
            original_places{i+1} = [];
            for j = 1 : length(temp)
                original_places{i+1} = union(original_places{i+1},data.Tr.Cells{temp(j)});
            end
            %                end
        else
            original_places{i+1} = original_places{i};
        end
    end
    
    options = cplexoptimset('cplex');
    options.display='off';
    options.lpmethod=1;
    message = sprintf('%s\n=======================================================',message);
    message = sprintf('%s\nProject the solution to the initial transition system with CPLEX',message);
    message = sprintf('%s\n=======================================================\n',message);
    [Pre,Post] = rmt_construct_PN(data.T.adj);
    m0 = data.T.m0;
    steps = 1;
    message = sprintf('%s\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0>eps*10^5)),mat2str(m0(m0>eps*10^5)));
    Run_cells = data.RO';%rmt_marking2places(m0);
    tic;
    for i = 1 : 2*data.intermediateMarkings
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
            [X,~,exitflag] = cplexlp(ones(1,size(Pre_new,2)+size(Pre_new,1)),[],[],...
                Aeq,beq,zeros(1,size(Pre_new,1)+size(Pre_new,2)),[],[],options);
            if ~(exitflag > 0)
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
    
    % Start part with GLPK
else
    ctype1='';
    ctype2='';
    for i = 1 : size(Aeq,1)
        ctype2 = sprintf('%sS',ctype2);
    end
    for i = 1 : size(A,1)
        ctype1 = sprintf('%sU',ctype1);
    end
    vartype = '';
    for i = 1 : 2*data.intermediateMarkings
        for j = 1 : size(Pre,1)
            vartype = sprintf('%sC',vartype); %put the markings as real
        end
        for j = 1 : size(Pre,2)
            vartype = sprintf('%sI',vartype); %put the sigma as integer
        end
    end
    tic;
    
    Atot = [Aeq; A];
    btot= [beq; b];
    ctype = [ctype2 ctype1];
    [xmin,f,exitflag] = glpk(cost,Atot,btot,zeros(1,size(A,2)),[],ctype,vartype);
    
    time = toc;
    if f==0%no solution
        uiwait(errordlg('Error solving the ILP. The problem may have no feasible solution. Increase k!','Robot Motion Toolbox','modal'));
        return;
    end
    message2 = sprintf('\nTime of solving the MILP: %g secs\n', time);
    message = sprintf('%s%s', message, message2);
    uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));
    
    message = sprintf('%s\n=======================================================',message);
    message = sprintf('%s\nInitial solution on the reduced transition system',message);
    message = sprintf('%s\n=======================================================\n',message);
    
    % After the optimization problem was solved, an
    % initial solution was obtained on the reduced system
    m0_old = m0;
    m0 = m0(1:nplaces_orig);
    m0_Buchi = m0_old(nplaces_orig+length(data.Tr.props)+1:end);
    m0_obs = m0_old(nplaces_orig+1:nplaces_orig+length(data.Tr.props));
    xm = xmin;
    xmin = [];
    ymin = [];
    fprintf(1,'\nInitial state of Buchi = %s',mat2str(find(m0_Buchi)));
    fprintf(1,'\n\tActive observations = %s',mat2str(find(m0_obs)));
    for i = 1 : 2*data.intermediateMarkings
        xmin = [xmin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+1:(i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig)];%marking of places modeling the team
        %fprintf(1,'Marking of places modeling the team at step %d: %s',)
        xmin = [xmin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+ntrans_orig)];%firing vector of places modeling the team
        
        ymin = [ymin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+length(data.Tr.props)+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1))];%marking of places modeling the Buchi
        ymin = [ymin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+ntrans_orig+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+size(Pre,2))];%firing vector of places modeling the buchi
        
        if (i/2 == round(i/2))
            trans_buchi=find([xm((i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+ntrans_orig+1:i*(size(Pre,1)+size(Pre,2)))]);
            input_place = find(Pre(:,trans_buchi+ntrans_orig)) ;
            input_place = input_place(input_place>nplaces_orig+length(data.Tr.props))-nplaces_orig-length(data.Tr.props); %take only the place of the buchi
            output_place = find(Post(:,trans_buchi+ntrans_orig));
            output_place = output_place(output_place>nplaces_orig+length(data.Tr.props))- nplaces_orig-length(data.Tr.props);
            fprintf(1,'\n Transition in Buchi from state %d to state %d with observation (%s)',input_place,output_place,mat2str(find([xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+1:(i-1)*(size(Pre,1)+size(Pre,2))++length(data.Tr.props)+nplaces_orig)])));
            fprintf(1,'\nState of Buchi in step %d = %s',i/2,mat2str(find([xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+length(data.Tr.props)+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1))])))
            %Decomment this part
            fprintf(1,'\n\tActive observations at step %d = %s',i/2,mat2str(find([xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+1:(i-1)*(size(Pre,1)+size(Pre,2))++length(data.Tr.props)+nplaces_orig)])));
        end
        
    end
    fprintf(1,'\n');
    message = sprintf('%s\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0>eps*10^5)),mat2str(m0(m0>eps*10^5)));
    original_places{1} = [];
    temp = rmt_marking2places(m0);
    for j = 1 : length(temp)
        original_places{1} = union(original_places{1},data.Tr.Cells{temp(j)});
    end
    nplaces = nplaces_orig;
    ntrans = ntrans_orig;
    
    for i = 1 : 2*data.intermediateMarkings
        m = xmin((i-1)*(nplaces+ntrans)+1 : (i-1)*(nplaces+ntrans)+nplaces);
        fire = xmin((i-1)*(nplaces+ntrans)+nplaces+1 : (i-1)*(nplaces+ntrans)+nplaces+ntrans);
        if (max(fire) > 0)
            message = sprintf('%s\n============STEP %d =============\n',message,i);
            message=sprintf('%s\nMarking [ %s ] = %s\n',message,mat2str(find(m>eps*10^5)),mat2str(m(m>eps*10^5)));
            message = sprintf('%s\nSigma [ %s ] = %s\n',message,mat2str(find(fire>eps*10^5)),mat2str(fire(fire>eps*10^5)));
            %                 if (reduced == 0)
            %                     Run_cells = [Run_cells, rmt_marking2places(m)];  %visited cells (rows)
            %                 else
            temp = rmt_marking2places(m);
            original_places{i+1} = [];
            for j = 1 : length(temp)
                original_places{i+1} = union(original_places{i+1},data.Tr.Cells{temp(j)});
            end
            %                end
        else
            original_places{i+1} = original_places{i};
        end
    end
    
    message = sprintf('%s\n=======================================================',message);
    message = sprintf('%s\nProject the solution to the initial transition system with GLPK',message);
    message = sprintf('%s\n=======================================================\n',message);
    [Pre,Post] = rmt_construct_PN(data.T.adj);
    m0 = data.T.m0;
    steps = 1;
    message = sprintf('%s\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0>eps*10^5)),mat2str(m0(m0>eps*10^5)));
    Run_cells = data.RO';%rmt_marking2places(m0);
    tic;
    for i = 1 : 2*data.intermediateMarkings
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
            %    Aeq,beq,zeros(1,size(Pre_new,1)+size(Pre_new,2)),[],[],options);
            ctype3='';
            for i = 1 : size(Aeq,1)
                ctype3 = sprintf('%sS',ctype3);
            end
            vartype3 = '';
            for j = 1 : size(Aeq,2)
                vartype3 = sprintf('%sC',vartype3); %put the sigma as integer
            end
            [X,~,exitflag] = glpk(ones(1,size(Pre_new,2)+size(Pre_new,1)),Aeq,beq,zeros(1,size(Pre_new,1)+size(Pre_new,2)),[],ctype3,vartype3,1);
            if f==0
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
end
time = toc;
message2 = sprintf('Total time for solving %d LPPs for projecting the solution: %g secs', ...
    2*data.intermediateMarkings, time);
uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));
message = sprintf('%s\n%s',message,message2);

cla(data.handle_env);
%%Execution monitoring strategy starts from here:
N_r = length(data.RO);
current_pos=cell(1,N_r);
for r=1:N_r
    current_pos{r}=data.initial{r};%R_trajs{r}(:,1); %current_pos - initial (current) position of robots (cell, current_pos{i} is a column vector with 2 rows)
end
rmt_represent_atomic_props(data.T.Vert,data.propositions);    %represent all atomic props and store handles

%independent runs of each robot (no synchronizations, because of our def. of Boolean formulae)
[Runs, ~] = rmt_robot_runs(data.T,Run_cells); %remove successive repetitions from individual discrete trajectories


rob_traj = rmt_rob_cont_traj(data.T,Runs,data.initial);    %continuous trajectory of each robot
message = sprintf('%s\nSOLUTION - runs of robots: \n',message);
for j = 1 : length(Runs)
    message = sprintf('%s\nRobot %d: ',message,j);
    temp = Runs{j};
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
