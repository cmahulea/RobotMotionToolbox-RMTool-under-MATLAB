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
% this script was implemented for TAC paper 2022
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
% data.T.OBS_set = rmt_clean_OBS_set(data.Nobstacles, data.formula, data.T.OBS_set);

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

% load('C:\Users\sofia\Work_H\Facultate\Doctorat\Articles\NwN_2022\MatlabWork\Buchi_ComplexMission.mat','B');
% load('C:\Users\sofia\Work_H\Facultate\Doctorat\Articles\NwN_2022\MatlabWork\workspace_sofia_1402.mat');

% load("C:\Users\sofia\Work_H\Facultate\Doctorat\Articles\NwN_2022\MatlabWork\workspace_3rob_complexMission_v1.mat", "B");
% load("C:\Users\sofia\Work_H\Facultate\Doctorat\Articles\NwN_2022\MatlabWork\workspace_3rob_complexMission_v2.mat", "B");
% load("C:\Users\sofia\Work_H\Facultate\Doctorat\Articles\NwN_2022\MatlabWork\workspace_3rob_complexMission_v3.mat", "B");
% load('C:\Users\sofia\Work_H\Facultate\Doctorat\Articles\NwN_2022\MatlabWork\workspace_6rob_complexMission.mat','B');


data.B=B;

set(gcf,'UserData',data);

% check if the self-loop of the initial state (when is equal with on final
% state) includes the initial active observations of the robots. It is
% assumed that the robots are initially placed in the free space
% (translated to last combination in the temp_obs variable)
flag_sisf_actobs = 0; % in this case, the computation of the prefix will not include the virtual transitions in the first step on firing Buchi transitions
sisf = intersect(B.S0,B.F);
if ~isempty(sisf)
    idx_sisf = find(B.F == sisf);
    if ~isempty(intersect(size(temp_obs,1),B.trans{idx_sisf,idx_sisf}))
        flag_sisf_actobs = 1; % include the virtual transition for the first step in Buchi PN
    end
end

Upp = (size(Pre,1)-1)*(length(B.S) - 1); %Upp is the upper bound of parameter data.optim.paramWith.interM --> k number of intermediate markings

% check condition for value k defined by the user to not be more than the
% upper bound
if (data.optim.paramWith.interM > Upp)
    choiceMenu = questdlg(sprintf('The number of intermedate markings %d is greater than the upper bound (PM - 1) x  (PB - 1) --> (number of places in Quotient PN - 1) x (number of states in Buchi - 1). The number of intermediate markings will be reduced to it''s upper bound!',...
        data.optim.paramWith.interM,Upp),'Robot Motion Toolbox - Path planning with PN models and Buchi included','Yes','Yes');
    if strcmpi(choiceMenu,'Yes')
        data.optim.paramWith.interM = Upp;
    end
end


% use new function to reduce transitions in Quontient Buchi PN
tic;
[Pre,Post,PreV, PostV, idxV, m0,final_places] = rmt_construct_PN_ltl(Pre,Post,m0,data.Tr.props, B);%final places - places corresponding to the final states in the Buchi automaton

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

idx_fs = 1; % take the first final state and search for a solution
flag_sol = 0; % solution is not projected

%(teta in paper) --> matrix containing all the bad solutions (transitions) from previous iterations on each column,
%as a result of MILP based on the reduced PN model, which could not be
%projected in the overall PN model of the environment

bad_sol_pr = []; % bad solutions for prefix
bad_sol_suf = []; % bad solutions for suffix
xmin_pref_prev = []; % memorize the previous xmin solution of the prefix, in case there is no otehr solution of the prefix, such that only the solution of the suffix to be modified.
bad_flag_gplk = [101:110 204:214];

counter = 0;
aux_pref = []; %the current solution of prefix considering sum(sigma_i), to be added in the bad_sol_pr if it's necessary
aux_suff = []; % idem as aux_pref

count_sol_not_proj = 0;
flag_bad_sol = 0; % flag which express if the solution could be projected (=0) or couldn't be projected (=1)

no_interMark = data.optim.paramWith.interM;

while flag_sol == 0 && ((no_interMark <= Upp) || (no_interMark > Upp && flag_bad_sol == 1))
    for idx_fs = 1:length(final_places)
        %% compute prefix for final state idx_fs
        tic;
        [A,b,Aeq,beq,cost] = rmt_construct_constraints_ltl_wBuchi(PreV,PostV,m0, ntrans_orig, ...
            2*no_interMark, final_places(idx_fs), idxV, bad_sol_pr, flag_sisf_actobs);
        tiempo = toc;
        
        message = sprintf('%s\n*****************************************************************',message);
        message = sprintf('%s\n\nFinal state %g\n',message, B.F(idx_fs));
        
        message_c = sprintf('%s-----------------------------------------------------------\n',message_c);
        message_c = sprintf('%s\nFinal state: %g\n',message_c,B.F(idx_fs));
        message_c = sprintf('%sTime of creating the optimization problem on quotient PN: %g secs\n',message_c,tiempo);
        time_c = time_c + tiempo;
        
        %         data = get(gcf,'UserData');
        % Part about analysis with Buchi Automaton
        %if strcmpi(get(data.optim.menuCplex,'Checked'),'on')
        %%%%%%%%%
        
        % umber of variables m_i sigma_i, for all i = 1:U (2k), without
        % considering the binary unknown variables
        no_var_msig = (size(Aeq,2) - 2*size(bad_sol_pr,2)*ntrans_orig);
        
        ctype='';
        for i = 1 : size(Aeq,1)
            ctype = sprintf('%sS',ctype);
        end
        for i = 1 : size(A,1)
            ctype = sprintf('%sU',ctype);
        end
        ub = []; % upper bound for intlinprog
        vartype = '';
        for i = 1 : no_var_msig/(size(PreV,1)+size(PreV,2))
            for j = 1 : size(PreV,1)
                vartype = sprintf('%sC',vartype); %put the markings as real
                ub = [ub inf];
            end
            for j = 1 : size(PreV,2)
                vartype = sprintf('%sI',vartype); %put the sigma as integer
                ub = [ub inf];
            end
            
        end
        
        for i = 1:2*size(bad_sol_pr,2)*ntrans_orig % number of binary unknown variables
            vartype = sprintf('%sB', vartype);
            ub = [ub 1];%upper bound with value 1 for the binary unknown variables
        end
        
        message = sprintf('%s\nTotal number of variables in the MILP problem (quotient PN): %d for %d intermediate markings',...
            message,size(Aeq,2),no_interMark);
        message = sprintf('%s\nThe optimization problem has %d equality contraints and %d inequality constraints (quotient PN).',...
            message, size(Aeq,1), size(A,1));
        
        tic;
        switch solver
            case 'cplex'
                [xmin_pref,fp,flp] = cplexmilp(cost,A,b,Aeq,beq,[],[],[],zeros(1,size(A,2)),[],vartype);
            case 'glpk'
                [xmin_pref,fp,flp] = glpk(cost,[Aeq; A],[beq; b],zeros(1,size(A,2)),[],ctype,vartype);
            case 'intlinprog'
                [xmin_pref,fp,flp] = intlinprog(cost, 1:length(cost), A, b, Aeq, beq, zeros(1,length(cost)), ub);
        end
        time = toc;
        
        flag_act_obs = 0;

        if ~isempty(fp)
            
            message = sprintf('%s\n\n---------------------- PREFIX -------------------',message);
            message = sprintf('%s\nFunction value for first MILP - prefix: %g \n', message, fp);
            message = sprintf('%s\nTime of solving the MILP - prefix (trajectory on quotient PN): %g secs\n', message, time);
            total_time = total_time + time;
            total_time_MILP = total_time_MILP + time;
            message_c = sprintf('%sTime of finding a path in the quotient PN with Buchi - prefix: %g secs\n',message_c,time);
            time_c = time_c + time;
            message = sprintf('%s\n=======================================================',message);
            message = sprintf('%s\nInitial solution on the reduced Petri net system',message);
            message = sprintf('%s\n=======================================================\n',message);
            
            % truncate the solution of MILP based of the unknown variables (z,w)
            % which appear for every bad solution
            xmin_pref = xmin_pref(1:size(A,2) - 2*size(bad_sol_pr,2)*ntrans_orig);
            
            % extract sum of all sigma_i in case the solution could not be
            % projected, and add it to bad_sol_pr
            aux_pref = zeros(ntrans_orig,1);
            for i = 1:2*no_interMark
                index1 = i*size(PreV,1) + (i-1)*size(PreV,2) + 1;
                index2 = index1 + ntrans_orig-1;
                aux_pref = aux_pref + xmin_pref(index1:index2);
            end
            
            % After the optimization problem was solved, an
            % initial solution was obtained on the reduced system
            % check the active observations after prefix
            [active_observations, possible_regions, number_of_robots, marking_new, message] = rmt_check_active_observations(xmin_pref,PreV,PostV,m0,data,nplaces_orig,ntrans_orig,message);
            % [active_observations, possible_regions, number_of_robots, marking_new, message] = rmt_check_active_observations(xmin_pref,PreV,PostV,m0,data,nplaces_orig,ntrans_orig,message);
            
            % modify the last active observations to combinations from temp_obs
            idx_act_temp = [];
            for idx_tempobs = 1:size(temp_obs,1)
                if length(intersect(active_observations{end},temp_obs(idx_tempobs,:))) == length(active_observations{end}) && ...
                        length(find(temp_obs(idx_tempobs,:))) == length(active_observations{end})
                    %         act_temp = [act_temp; temp_obs(idx_tempobs,:)]; % save observations which include active observations
                    idx_act_temp = idx_tempobs; % save index coresponding to temp_obs
                end
            end
            
            % check if the last active observations are a subset of observations in the
            % self-loop of the final state
%             flag_act_obs = 0;
            temp_fs = B.F(idx_fs);
            
            for idx_obs = 1:size(B.new_trans{temp_fs,temp_fs},1)
                if B.new_trans{temp_fs,temp_fs} == Inf | ~isempty(intersect(idx_act_temp, B.trans{temp_fs,temp_fs}))
                    flag_act_obs = 1; % final state has self-loop on True or the active observations are included in the self-loop
                end
            end
            
            % update initial marking for MILP 1.2
            No_obstacles = data.Nobstacles;
            
            m0_fs = m0;
            m0_fs(1:length(data.Tr.Cells)) = marking_new; %put the final marking for the Q_PN places
            
            %put the final active observations on 1
            idx_obs = data.Tr.obs(find(marking_new));
            temp_m0_fs = m0_fs(length(data.Tr.Cells)+1:length(data.Tr.Cells) + No_obstacles);
            temp_ao = [];
            % idx_obs(idx_ao) = index from data.Tr.OBS_set with regards to the
            % combination for one robot
            for idx_ao = 1:length(idx_obs)
                temp_ao = [temp_ao data.Tr.OBS_set(idx_obs(idx_ao),find(data.Tr.OBS_set(idx_obs(idx_ao),:)))];
            end
            real_temp_ao = temp_ao(find(temp_ao <= No_obstacles)); % the active observation need to be represented by ROI and not the free space (the free space = No_obstacles + 1)
            temp_m0_fs(real_temp_ao) = 1;
            m0_fs(length(data.Tr.Cells)+1:length(data.Tr.Cells) + No_obstacles) = temp_m0_fs;
            
            %put the negated observations = number of robots
            temp_m0_fs = m0_fs(length(data.Tr.Cells)+No_obstacles+1:length(data.Tr.Cells) + 2*No_obstacles);
            temp_m0_fs(real_temp_ao) = 0;
            m0_fs(length(data.Tr.Cells)+No_obstacles+1:length(data.Tr.Cells) + 2*No_obstacles) = temp_m0_fs;
            
            m0_fs(final_places(idx_fs)) = 1;
            m0_fs(end - length(B.S) + 1) = 0;
            
        elseif isempty(fp) && no_interMark <= Upp
            uiwait(errordlg('Error solving the ILP on quotient PN: a solution different than the previous one could not be found. The algorithm will consider the previous solution of the PREFIX.',...
                'Robot Motion Toolbox','modal'));
        elseif isempty(fp) && no_interMark > Upp %no solution
            uiwait(errordlg('Error solving the ILP on quotient PN. The problem may have no feasible solution!',...
                'Robot Motion Toolbox','modal'));
            return;
            
        end

        %% compute suffix
        
        % if flag_act_obs == 0 OR if the prefix could not be computed such
        % that the current solution is different than the previous one,
        % solve MILP 1.2 for suffix. In this case, the solution of the
        % prefix is considered as the previous one
        
        if flag_act_obs == 0 || (flp < 1 || ~isempty(find(bad_flag_gplk == flp)))
            [A,b,Aeq,beq,cost] = rmt_construct_constraints_ltl_wBuchi(PreV,PostV,m0_fs, ntrans_orig, ...
                2*no_interMark, final_places(idx_fs),idxV,bad_sol_suf);
            
            %             data = get(gcf,'UserData');
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
            ub = []; % upper bound for intlinprog
            vartype = '';
            for i = 1 : no_var_msig/(size(PreV,1)+size(PreV,2))
                for j = 1 : size(PreV,1)
                    vartype = sprintf('%sC',vartype); %put the markings as real
                    ub = [ub inf];
                end
                for j = 1 : size(PreV,2)
                    vartype = sprintf('%sI',vartype); %put the sigma as integer
                    ub = [ub inf];
                end
                
            end
            
            for i = 1:2*size(bad_sol_pr,2)*ntrans_orig % number of binary unknown variables
                vartype = sprintf('%sB', vartype);
                ub = [ub 1];%upper bound with value 1 for the binary unknown variables
            end
            

            message = sprintf('%s\nTotal number of variables in the MILP problem (quotient PN): %d for %d intermediate markings',...
                message,size(Aeq,2),no_interMark);
            message = sprintf('%s\nThe optimization problem has %d equality contraints and %d inequality constraints (quotient PN).',...
                message, size(Aeq,1), size(A,1));
            
            tic;
            switch solver
                case 'cplex'
                    [xmin_suff,f_suff,fls] = cplexmilp(cost,A,b,Aeq,beq,[],[],[],zeros(1,size(A,2)),[],vartype);
                case 'glpk'
                    [xmin_suff,f_suff,~] = glpk(cost,[Aeq; A],[beq; b],zeros(1,size(A,2)),[],ctype,vartype);
                    % if it's no solution, f_suff == 0
                case 'intlinprog'
                    [xmin_suff,f_suff,~] = intlinprog(cost, 1:length(cost), A, b, Aeq, beq, zeros(1,length(cost)), ub);
            end
            time = toc;
            total_time_MILP = total_time_MILP + time;
            if isempty(f_suff) || f_suff == 0 % no solution
                uiwait(errordlg('Error solving the ILP on quotient PN. The problem may have no feasible solution. Increase k(Setup -> Parameters for MILP PN with Buchi)!',...
                    'Robot Motion Toolbox','modal'));
                return;
            end
            message = sprintf('%s\n\n---------------------- SUFFIX --------------------',message);
            message = sprintf('%s\nFunction value for first MILP - suffix: %g \n', message, f_suff);
            message = sprintf('%s\nTime of solving the MILP - suffix (trajectory on quotient PN): %g secs\n', message, time);
            total_time = total_time + time;
            message_c = sprintf('%sTime of finding a path in the quotient PN with Buchi - suffix: %g secs\n',message_c,time);
            time_c = time_c + time;
            message = sprintf('%s\n=======================================================',message);
            message = sprintf('%s\nInitial solution on the reduced Petri net system',message);
            message = sprintf('%s\n=======================================================\n',message);
            
            % truncate the solution of MILP based of the unknown variables (z,w)
            % which appear for every bad solution
            xmin_suff = xmin_suff(1:size(A,2) - 2*size(bad_sol_pr,2)*ntrans_orig);
            
            % extract sum of all sigma_i in case the solution could not be
            % projected, and add it to bad_sol_pr
            
            
            aux_suff = zeros(ntrans_orig,1);
            for i = 1:2*no_interMark
                index1 = i*size(PreV,1) + (i-1)*size(PreV,2) + 1;
                index2 = index1 + ntrans_orig-1;
                aux_suff = aux_suff + xmin_suff(index1:index2);
            end
            
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
        
        %% second MILP - project the solution
        
        [PreP,PostP] = rmt_construct_PN(data.T.adj);
        m0_ps = data.T.m0;
        nplaces = size(PostP,1);
        ntrans = size(PostP,2);
        steps = 1;
        message = sprintf('%s\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0_ps>eps*10^5)),mat2str(m0_ps(m0_ps>eps*10^5)));
        
        tic;
        
        [A,b,Aeq,beq,cost] = rmt_construct_constraints_ltl_project_sol(PreP,PostP,m0_ps, number_of_robots,...
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
            for j = 1 : size(PreP,1)
                vartype = sprintf('%sC',vartype); %put the markings as real
            end
            for j = 1 : size(PreP,2)
                vartype = sprintf('%sI',vartype); %put the sigma as integer
            end
        end
        
        
        tic;
        switch solver
            case 'cplex'
                [X,f,flag] = cplexlp(cost,A,b,Aeq,beq,zeros(1,size(Aeq,2)),[]);
            case 'glpk'
                [X,f,flag] = glpk(cost,[Aeq; A],[beq; b],zeros(1,size(A,2)),[],ctype,vartype);
            case 'intlinprog'
                [X,f,flag] = intlinprog(cost, 1:length(cost), A, b, Aeq, beq, zeros(1,length(cost)), []);
                
        end
        time = toc;
        
        %% check if the solution of the reduced model could be projected into the full PN model or not
        
        if flag < 1 || ~isempty(find(bad_flag_gplk == flag))
            
            flag_bad_sol = 1;
            bad_sol_pr = [bad_sol_pr aux_pref];
            bad_sol_suf = [bad_sol_suf aux_suff];
            count_sol_not_proj = count_sol_not_proj + 1;
            message = sprintf('%s\n\n ----*----*----*----*----*----*----*----*----*----*----*\n\n',message);
            message = sprintf('%s\nThe solution of the reduced model could not be projected for %d times!!! \n', message,count_sol_not_proj);
            
        else
            flag_sol = 1;
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
            
            % save all
            vect_fmin = [vect_fmin f];
            cell_X{idx_fs} = X;
            
            %             break; % solution is found and we can exit the while
            
        end
        
    end
    %%%% Check interM value!!!!!! (k or 2k)???
    no_interMark = no_interMark + 1
    
end

if flag_sol == 0
    uiwait(errordlg('Error solving LPP to project the solution when !','Robot Motion Toolbox','modal'));
    return;
else
    % memorize final state with the minimum cost function value
    [temp_fmin, idx_fmin] = min(vect_fmin);
    
    message = sprintf('%s\n\n______________________________________________________\n\n Selected final state: %g\n Cost function value: %g \n', message,B.F(idx_fmin), temp_fmin);
    message = sprintf('%s\n\nTotal time for MILPs: %g\n', message,total_time_MILP);
    
    Run_cells = data.RO';
    [message,Run_cells] = rmt_path_planning_ltl_with_buchi_trajectories(cell_X{idx_fmin},PreP,PostP,Run_cells,Run_cells,message);
    
    %% Plot trajectories - Execution monitoring strategy starts from here:
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
    
    rob_color={'g','c','m','k','y','r','b','g','c','m','k','y','r','b','g','c'};
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
