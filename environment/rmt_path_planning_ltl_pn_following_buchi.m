%    This is part of RMTool - Robot Motion Toolbox, for Matlab 2010b or newer.
%
%    Copyright (C) 2019 RMTool developing team. For people, details and citing
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
%   Last modification May, 2020.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function rmt_path_planning_ltl_pn_following_buchi

%Path-planning with LTL specifications and Petri net models following paths
%in Buchi automaton

disp('Computing trajectories (PN models folowing paths in Buchi). Please wait...');
data = get(gcf,'UserData');
N_p = data.Nobstacles;%number of regions of interest
N_r = length(data.RO); %number of robots
Obs = rmt_observation_set_new(data.T.OBS_set,N_p,N_r); %observations - power set of \Pi
total_time = 0;
time_c = 0;
message_c = sprintf('----------------------------------------------------------------------------------------------------\n');
message_c = sprintf('%s---Path planning using Petri net models following rund in Buchi---\n',message_c);
message_c = sprintf('%s----------------------------------------------------------------------------------------------------\n',message_c);

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
tic;
[Pre,Post] = rmt_construct_PN(data.T.adj);
m0 = data.T.m0;
tiempo = toc;
message = sprintf('Petri net system has %d places and %d transitions\nTime spent to create the PN system: %g secs',...
    size(Pre,1),size(Pre,2),tiempo);
message_c = sprintf('%sTime of generating the PN model of the team: %g secs\n',message_c,tiempo);
time_c = time_c + tiempo;

%nplaces = size(Pre,1);
%ntrans = size(Pre,2);

%***Buchi automaton for formula, with elements of power set of \Pi (Obs) on transitions***%
tic;
B = rmt_create_buchi(data.formula, Obs);
tiempo = toc;
message = sprintf('%s\nBuchi automaton has %d states\nTime spent to create Buchi: %g secs',...
    message,length(B.S),tiempo);
message_c = sprintf('%sTime of generating the Buchi automaton: %g secs\n',message_c,tiempo);
time_c = time_c + tiempo;

% check if the initial positions of the robots violate the LTL formula
[temp_message] = rmt_check_initial_active_obs(data,B,Obs,m0,Pre);

message = sprintf('%s\n%s\n',message,temp_message);

%%***solution's main part: choose a path in B and try to follow it with observations of PN
tic;
adj_B = rmt_graph_for_Buchi(B);
paths_B = {};
index_suffix = [];
for ii = 1 : length(B.S0)
    for jj = 1 : length(B.F)
        [paths_B_pref, ~] = kShortestPath(adj_B,B.S0(ii),B.F(jj), data.optim.param.kappa); %use k shortest paths, with k = kappa
        Ssf = find(adj_B(:,B.F(jj)));%find all transitions that have transitions to the terminal state
        for oo = length(Ssf): -1 : 1
            if ~isfinite(adj_B(Ssf(oo),B.F(jj)))
                Ssf(oo) = [];
            end
        end
        for kk = 1 : length(Ssf)
            if ((B.F(jj) == Ssf(kk)) && (adj_B(B.F(jj),B.F(jj))==1))
                if (isempty(paths_B_pref) &&  length(union(B.S0(ii),B.F))==length(B.F)) %if the initial state is also final one
                    index_suffix(length(paths_B)+1) = 2;
                    paths_B{length(paths_B)+1} = [B.F(jj) B.F(jj)];
                end
                for ll = 1 : length(paths_B_pref)
                    index_suffix(length(paths_B)+1) = length(paths_B_pref{ll}) +1;
                    paths_B{length(paths_B)+1} = [paths_B_pref{ll} B.F(jj)];
                end
            elseif (B.F(jj) ~= Ssf(kk))
                [paths_B_suff, ~] = kShortestPath(adj_B,B.F(jj),Ssf(kk), ...
                    ceil(data.optim.param.kappa/length(Ssf))); %use k shortest paths from final state to a state in Ssf
                for mm = 1 : length(paths_B_suff)
                    temp = paths_B_suff{mm};
                    temp = [temp(2:end) temp(1)];
                    paths_B_suff{mm} = temp;
                end
                if (isempty(paths_B_pref) &&  length(union(B.S0(ii),B.F))==length(B.F)) %if the initial state is also final one
                    for mm = 1 : length(paths_B_suff)
                        index_suffix(length(paths_B)+1) = 2;
                        paths_B{length(paths_B)+1} = [B.F(jj) paths_B_suff{mm}];
                    end
                end
                for ll = 1 : length(paths_B_pref)
                    for mm = 1 : length(paths_B_suff)
                        index_suffix(length(paths_B)+1) = length(paths_B_pref{ll}) +1;
                        paths_B{length(paths_B)+1} = [paths_B_pref{ll} paths_B_suff{mm}];
                    end
                end
            end
        end
    end
end

tiempo = toc;
message = sprintf('%s\nComputing %d paths in Buchi\nTime spent to compute all paths: %g secs',...
    message,length(paths_B),tiempo);
message_c = sprintf('%sTime of finding %d accepted paths in Buchi: %g secs\n',message_c,length(paths_B),tiempo);
time_c = time_c + tiempo;

%try to follow a path in Buchi
feasible_path_B=0;
message2 = '';
time_paths = 0;
Rob_positions = data.RO;   %initial (current) robot positions for testing optimization solutions
message = sprintf('%s\n\nInitial regions of robots: ',message);
for j = 1 : length(Rob_positions)-1
    message = sprintf('%s p%d,',message,Rob_positions(j));
end
message = sprintf('%s p%d.\n',message,Rob_positions(length(Rob_positions)));

active_obs=[];
props = data.T.props;
for j = 1 : length(Rob_positions)
    for ll = 1 : length(props)
        temp = props{ll};
        if ~isempty(intersect(Rob_positions(j),temp))
            active_obs = [active_obs ll];
        end
    end
end
active_obs = unique(active_obs);
if isempty(active_obs)
    message = sprintf('%sNo active observations at initial position.\n ',message);
else
    message = sprintf('%sActive observations at initial position: ',message);
    for i = 1 : length(active_obs)-1
        message = sprintf('%s y%d,',message,active_obs(i));
    end
    message = sprintf('%s y%d\n',message,active_obs(length(active_obs)));
end


for p_B=1:length(paths_B)
    message = sprintf('%s\nTrying to follow path %d in Buchi',message,p_B);
    path_B = paths_B{p_B};  %chosen path in Buchi
    %     message2 = sprintf('\n\n PATH_%d in Buchi is: %s\n',p_B,num2str(path_B));
    message = sprintf('%s\n\n PATH_%d in Buchi is: %s\n',message,p_B,num2str(path_B));
    %initialization
    feasible_path_B=1;  %assume current path in Buchi is feasible
    Rob_positions = data.RO;   %initial (current) robot positions for testing optimization solutions
    PN_marking = m0;    %initial (current) PN marking
    Rob_places = cell(1,N_r);
    Rob_trans = cell(1,N_r);
    Rob_synchronize = cell(1,N_r);
    for j=1:N_r
        Rob_places{j}=Rob_positions(j); %initial position
        Rob_trans{j}=[];
        Rob_synchronize{j}=[];
    end
    tic;
    for i=1:(length(path_B)-1)  %find traj in PN s.t. Buchi goes from state path_B(i) to path_B(i+1), without getting another observation that leaves path_B(i) to other state
        message = sprintf('%s\n  Try to ENABLE the current transition of Buchi: %g -> %g.\n',...
            message,path_B(i),path_B(i+1));
        obs_fin=B.trans{path_B(i),path_B(i+1)}; %row indices in Obs for desired observations in final marking
        % obs_feasible = union(B.trans{path_B(i),path_B(i)},B.trans{path_B(i),path_B(i+1)}); %not correct to take union; indices in Obs for all observations that do not leave sequence of states [path_B(i) -> path_B(i+1)] from Buchi
        obs_feasible = B.trans{path_B(i),path_B(i)};    %feasible observations on trajectory (self-loop in B); set of obs may not include set of final obs (otherwise, errors may result - e.g. formulas like '!p2 U (p1 & p3)')
        
        %*** 1) try only desired observations in final marking: those that enable transition path_B(i) -> path_B(i+1) in Buchi
        message = sprintf('%s\n\tSolving ILP 1 with solver %s ... ',message,solver);
        
        switch solver
            case 'glpk'
                [cost, matrix_A, vector_b, lb, ub, ctype, vartype, sense] = rmt_constraints_PN_obs_new(Pre, Post, PN_marking, ...
                    data.T.props, {B.new_trans{path_B(i),path_B(i+1)},[]}, 'final', data.optim.param.intMarkings, ...%B.new_trans,
                    data.optim.param.alpha, data.optim.param.beta, data.optim.param.gamma, 'glpk'); %constraints for ILP for PN
                [xmin, fmin, status, extra] = glpk(cost, matrix_A, vector_b, lb, ub, ctype, vartype, sense,data.optim.options_glpk);   %optimization with glpk
                
            case 'intlinprog'
                [cost, intcon, A, b, Aeq, beq, lb, ub] = rmt_constraints_PN_obs_new(Pre, Post, PN_marking, ...
                    data.T.props, {B.new_trans{path_B(i),path_B(i+1)},[]}, 'final', data.optim.param.intMarkings, ...%B.new_trans,
                    data.optim.param.alpha, data.optim.param.beta, data.optim.param.gamma, 'intlinprog'); %constraints for ILP for PN
                A_sparse=sparse(A); %can use sparse for large matrices
                Aeq_sparse=sparse(Aeq);
                [xmin, fmin, status, extra] = intlinprog(cost, intcon, A_sparse, b, Aeq_sparse, beq, lb, ub, data.optim.options_milp);   %optimization with intlinprog
                
            case 'cplex'
                [cost, A, b, Aeq, beq, lb, ub, vartype]  = rmt_constraints_PN_obs_new(Pre, Post, PN_marking, ...
                    data.T.props, {B.new_trans{path_B(i),path_B(i+1)},[]}, 'final', data.optim.param.intMarkings, ...%B.new_trans,
                    data.optim.param.alpha, data.optim.param.beta, data.optim.param.gamma, 'cplex'); %constraints for ILP for PN
                A_sparse=sparse(A); %can use sparse for large matrices            Pre, Post, m0,          props,       Obs, obs_type, set_ind_fin, set_ind_traj, k, alpha, beta, gamma, solver
                Aeq_sparse=sparse(Aeq);
                [xmin, fmin, status, extra] = cplexmilp(cost, A, b, Aeq, beq, [],[],[], lb, ub, ...
                    vartype, [], []);   %optimization with cplexmilp
        end
        
        prec=0.5e-5;    %additional tests for solution feasibility
        switch solver
            case 'glpk'
                if ((status == 5) && (max(matrix_A*xmin-vector_b)>prec || max(lb-xmin)>prec)) %this happened for too large M in constraints_PN_obs, for x_i and x_i(t) variables
                    message2 = sprintf('%s\nWrong solution returned by GLPK -> It doesn''t satisfy constraints! Correctness of results is not guaranteed.\n',message2);
                else
                    message2 = ' ';
                end
            case 'intlinprog' %this didn't occur; at least an error signaled by Matlab in intlinprog, caused actually by too large M in constraints_PN_obs, for x_i and x_i(t) variables
                if ((status == 1) && (max(A*xmin-b)>prec || max(abs(Aeq*xmin-beq))>prec || max(lb-xmin)>prec || max(xmin-ub)>prec))
                    message2 = sprintf('%s\nWrong solution returned by INTLINPROG -> It doesn''t satisfy constraints! Correctness of results is not guaranteed.\n',message2);
                else
                    message2 = ' ';
                end
                
            case 'cplex' %this didn't occur
                if ((status >= 0) && (max(A*xmin-b)>prec || max(abs(Aeq*xmin-beq))>prec || max(lb-xmin)>prec))
                    message2 = sprintf('%s\nWrong solution returned by CPLEX -> It doesn''t satisfy constraints! Correctness of results is not guaranteed.\n',message2);
                else
                    message2 = ' ';
                end
        end
        message = [message message2];
        
        %test solution feasibility (non-spurious sigma and generated observables do not leave current transition in Buchi)
        [message2,feasible_sol , marking_final, Rob_positions_final, Rob_places_temp, Rob_trans_temp, ...
            Rob_synchronize_temp, min_k1] = rmt_test_solution_feasibility(message2,Pre, Post, PN_marking, ...
            data.T.props, Obs, 'final', obs_fin, obs_feasible, data.optim.param.intMarkings, solver, xmin, ...
            status, Rob_positions);
        message = [message message2];
        if (feasible_sol == 0)
            message = sprintf('%s\tUnfeasible solution with ILP1 when trying to take current transition (s%g -> s%g) in Buchi automaton!\n',...
                message,path_B(i),path_B(i+1));
        end
        if (feasible_sol==1) %ILP1 gives good result, go to next transition in Buchi
            Rob_positions = Rob_positions_final;
            PN_marking = marking_final;
            for j=1:N_r
                Rob_synchronize{j}=[Rob_synchronize{j} , Rob_synchronize_temp{j} + length(Rob_places{j})-1]; %when to synchronize (number of visited place)
                Rob_places_temp{j}(1)=[];  %initial position already included
                Rob_places{j}=[Rob_places{j} , Rob_places_temp{j}];
                Rob_trans{j}=[Rob_trans{j} , Rob_trans_temp{j}];
            end            
            continue;
        end
        
        
        %*** 2) if feasible_sol==0: try feasible observations for reached markings given by sigma: those that either loop in path_B(i), or go from path_B(i) to path_B(i+1)
        %         message2 = sprintf('%s\n\t Solving ILP 2 with solver %s ... ',message2,solver);
        message = sprintf('%s\n\tSolving ILP 2 with solver %s ... ',message,solver);
        
        switch solver
            case 'glpk'
                [cost, matrix_A, vector_b, lb, ub, ctype, vartype, sense] = rmt_constraints_PN_obs_new(Pre, Post, PN_marking, ...
                    data.T.props, {B.new_trans{path_B(i),path_B(i+1)},B.new_trans{path_B(i),path_B(i)}},'trajectory', data.optim.param.intMarkings,...
                    data.optim.param.alpha, data.optim.param.beta, data.optim.param.gamma, 'glpk'); %constraints for ILP for PN
                [xmin, fmin, status, extra] = glpk(cost, matrix_A, vector_b, lb, ub, ctype, vartype, sense,data.optim.options_glpk);   %optimization with GLPK
            case 'intlinprog'
                [cost, intcon, A, b, Aeq, beq, lb, ub] = rmt_constraints_PN_obs_new(Pre, Post, PN_marking, ...
                    data.T.props, {B.new_trans{path_B(i),path_B(i+1)},B.new_trans{path_B(i),path_B(i)}},'trajectory', data.optim.param.intMarkings,...
                    data.optim.param.alpha, data.optim.param.beta, data.optim.param.gamma, 'intlinprog'); %constraints for ILP for PN
                A_sparse=sparse(A); %can use sparse for large matrices
                Aeq_sparse=sparse(Aeq);
                [xmin, fmin, status, extra] = intlinprog(cost, intcon, A_sparse, b, Aeq_sparse, beq, lb, ub, data.optim.options_milp);   %optimization with intlinprog
                
            case 'cplex'
                [cost, A, b, Aeq, beq, lb, ub, vartype]  = rmt_constraints_PN_obs_new(Pre, Post, PN_marking, ...
                    data.T.props, {B.new_trans{path_B(i),path_B(i+1)},B.new_trans{path_B(i),path_B(i)}},'trajectory', data.optim.param.intMarkings,...
                    data.optim.param.alpha, data.optim.param.beta, data.optim.param.gamma, 'cplex'); %constraints for ILP for PN
                A_sparse=sparse(A); %can use sparse for large matrices
                Aeq_sparse=sparse(Aeq);
                [xmin, fmin, status, extra] = cplexmilp(cost, A, b, Aeq, beq, [],[],[], lb, ub, vartype, [], []);   %optimization with cplexmilp
        end
        
        prec=0.5e-5;    %additional tests for solution feasibility
        switch solver
            case 'glpk'
                if (status == 5) && (max(matrix_A*xmin-vector_b)>prec || max(lb-xmin)>prec) %this happened for too large M in constraints_PN_obs, for x_i and x_i(t) variables
                    message2 = sprintf('%s\nWrong solution returned by GLPK -> It doesn''t satisfy constraints! Correctness of results is not guaranteed.\n',message2);
                else
                    message2 = ' ';
                end
            case 'intlinprog' %this didn't occur; at least an error signaled by Matlab in intlinprog, caused actually by too large M in constraints_PN_obs, for x_i and x_i(t) variables
                if (status == 1) && (max(A*xmin-b)>prec || max(abs(Aeq*xmin-beq))>prec || max(lb-xmin)>prec || max(xmin-ub)>prec)
                    message2 = sprintf('%s\nWrong solution returned by INTLINPROG -> It doesn''t satisfy constraints! Correctness of results is not guaranteed.\n',message2);
                else
                    message2 = ' ';
                end
                
            case 'cplex' %this didn't occur
                if (status >= 0) && (max(A*xmin-b)>prec || max(abs(Aeq*xmin-beq))>prec || max(lb-xmin)>prec)
                    message2 = sprintf('%s\nWrong solution returned by CPLEX -> It doesn''t satisfy constraints! Correctness of results is not guaranteed.\n',message2);
                else
                    message2 = ' ';
                end
        end
        message = [message message2];
        
        %test solution feasibility (non-spurious sigma and generated observables do not leave current transition in Buchi)
        [message2,feasible_sol , marking_final, Rob_positions_final, Rob_places_temp, Rob_trans_temp, ...
            Rob_synchronize_temp, min_k2] = rmt_test_solution_feasibility(message2,Pre, Post, PN_marking, ...
            data.T.props, Obs, 'trajectory', obs_fin, obs_feasible, data.optim.param.intMarkings, ...
            solver, xmin, status, Rob_positions);
        message = [message message2];
        if feasible_sol == 0
            %                 message2 = sprintf('%s\nUnfeasible solution with ILP2 \n \n IMPOSSIBLE to take current transition (s%g -> s%g) in Buchi automaton!\n',...
            %             message2,path_B(i),path_B(i+1));
            message = sprintf('%s\nUnfeasible solution with ILP2 \n \n IMPOSSIBLE to take current transition (s%g -> s%g) in Buchi automaton!\n',...
                message,path_B(i),path_B(i+1));
        end
        if feasible_sol==1 %ILP2 gives good result, go to next transition in Buchi
            Rob_positions = Rob_positions_final;
            PN_marking = marking_final;
            for j=1:N_r
                Rob_synchronize{j}=[Rob_synchronize{j} , Rob_synchronize_temp{j} + length(Rob_places{j})-1]; %when to synchronize (number of visited place)
                Rob_places_temp{j}(1)=[];  %initial position already included
                Rob_places{j}=[Rob_places{j} , Rob_places_temp{j}];
                Rob_trans{j}=[Rob_trans{j} , Rob_trans_temp{j}];
            end
            continue;
        end
        feasible_path_B=0; %current path in Buchi is not feasible
        tiempo = toc;
        time_paths = time_paths + tiempo;
        total_time = total_time + tiempo;
        %         message2 = sprintf('%s\n\t Path_%d NOT feasible. Time solving the ILPs trying to follow it: %g seconds. \n \n',...
        %             message2,p_B,tiempo);
        
        
        message = sprintf('%s\n\t Path_%d NOT feasible. Time solving the ILPs trying to follow it: %g seconds. \n \n',...
            message,p_B,tiempo);
        %         for ii = 1:length(paths_B{p_B})
        %              message = sprintf('%s\t %d  ',...
        %             message,paths_B{p_B}(ii));
        %         end
        break;
    end
    
    if feasible_path_B==1   %path in B was followed
        tiempo = toc;
        %         message2 = sprintf('%s\n\t Path_%d is feasible. Time solving all ILPs: %g seconds. \n ',...
        %             message2,p_B,tiempo);
        message = sprintf('%s\n\t Path_%d is feasible.\n \t Time solving all ILPs: %g seconds. \n ',...
            message,p_B,tiempo);
        time_paths = time_paths + tiempo;
        break;
    end
end
message_c = sprintf('%sTime of finding the path(s): %g secs\n',message_c,time_paths);

if feasible_path_B==0   %no path in B could be followed
    %     message2 = sprintf('%s\nNO SOLUTION! No path in B could be followed. \n\tPossible reasons:\n\t\t-the problem is unfeasible (impossible to satisfy task);\n\t\t-too few paths in Buchi;\n\t\t-too small number of intermediate PN markings.\n',message2);
    %     message2 = sprintf('%s\nTotal time of solving all MILPS: %g seconds\n',message2,total_time);
    message = sprintf('%sNO SOLUTION! No path in B could be followed.\nTotal time of solving all ILPs: %g seconds\n',...
        message,tiempo);
    message = sprintf('%s\nTotal time of solving all MILPS: %g seconds\n',message,total_time);
    
else %plot robot trajectories
    [rob_traj,synch_points] = rmt_rob_cont_traj_synch(data.T,Rob_places,Rob_synchronize,data.initial);    %continuous trajectory of each robot
    
    rob_color={'g','r','b','m','k','y','c','g','r','b','m','k','y','c','g','r','b','m','k','y','c'};  %colors of robots
    data.trajectory = rob_traj;
    f_value = 0;
    for ii = 1:length(Rob_places)
        f_value = f_value + length(Rob_places{ii});
        message = sprintf('%s\n Path of robot %d: ',message,ii);
        for jj = 1:length(Rob_places{ii})
            message = sprintf('%s %d ',message, Rob_places{ii}(jj));
        end
    end
    message = sprintf('%s \n\n Function value (movements of the robots): %d ', message, f_value - N_r);
    for r=1:N_r    %plot trajectories of robots
        plot(rob_traj{r}(1,1),rob_traj{r}(2,1),rob_color{mod(r-1,length(rob_color))+1},'Marker','o','LineWidth',2);
        plot(rob_traj{r}(1,:),rob_traj{r}(2,:),rob_color{mod(r-1,length(rob_color))+1},'LineWidth',4);
        plot(rob_traj{r}(1,end),rob_traj{r}(2,end),rob_color{mod(r-1,length(rob_color))+1},'Marker','x','LineWidth',2);
        for jj = 1:size(synch_points{r},2)
            plot(synch_points{r}(1,jj),synch_points{r}(2,jj),'k','Marker','*','MarkerSize',5);
        end
    end
end
time_c = time_c + time_paths;
message_c = sprintf('%sTotal time: %g secs\n',message_c,time_c);
disp(message_c);

button = questdlg('Save the details to a text file?','Robot Motion Toolbox');
if strcmpi(button,'Yes')
    [filename, pathname] = uiputfile('*.txt', 'Save experiments as');
    fileID = fopen(fullfile(pathname, filename),'w');
    %     fprintf(fileID,'%s\n\nDETAILED STEPS %s',message,message2);
    fprintf(fileID,'\n\nDETAILED STEPS \n%s',message);
    fclose(fileID);
end

