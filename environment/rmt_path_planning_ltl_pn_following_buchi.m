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
%   Last modification November 10, 2018.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function rmt_path_planning_ltl_pn_following_buchi

%Path-planning with LTL specifications and Petri net models following runs
%in Buchi automaton

data = get(gcf,'UserData');
N_p = data.Nobstacles;%number of regions of interest
N_r = length(data.RO); %number of robots
Obs = rmt_observation_set_new(data.T.OBS_set,N_p,N_r); %observations - power set of \Pi
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
[Pre,Post] = rmt_construct_PN(data.T.adj);
m0 = data.T.m0;

%nplaces = size(Pre,1);
%ntrans = size(Pre,2); 

%***Buchi automaton for formula, with elements of power set of \Pi (Obs) on transitions***%
B = rmt_create_buchi(data.formula, Obs);

%%***solution's main part: choose a path in B and try to follow it with observations of PN
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
 
%try to follow a path in Buchi
feasible_path_B=0;

for p_B=1:length(paths_B)
    path_B = paths_B{p_B};  %chosen path in Buchi
    fprintf('\n\nCURRENT PATH in Buchi: %s\n',num2str(path_B));
    
    min_k=3;    %predefined minimum number of intermediate markings (take value of k for ILP3 from 1*sigma1 (ILP 1), 1*sigma2 (ILP2) and min_k)
    
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
    
    for i=1:(length(path_B)-1)  %find traj in PN s.t. Buchi goes from state path_B(i) to path_B(i+1), without getting another observation that leaves path_B(i) to other state
        fprintf('\n  Try to ENABLE the current transition of Buchi: %g -> %g:\n',path_B(i),path_B(i+1));
        obs_fin=B.trans{path_B(i),path_B(i+1)}; %row indices in Obs for desired observations in final marking
        % obs_feasible = union(B.trans{path_B(i),path_B(i)},B.trans{path_B(i),path_B(i+1)}); %not correct to take union; indices in Obs for all observations that do not leave sequence of states [path_B(i) -> path_B(i+1)] from Buchi
        obs_feasible = B.trans{path_B(i),path_B(i)};    %feasible observations on trajectory (self-loop in B); set of obs may not include set of final obs (otherwise, errors may result - e.g. formulas like '!p2 U (p1 & p3)')
        
        %*** 1) try only desired observations in final marking: those that enable transition path_B(i) -> path_B(i+1) in Buchi
        fprintf('\n\t Solving ILP 1 with solver %s ... ',solver);
        switch solver
            case 'glpk'
                [cost, matrix_A, vector_b, lb, ub, ctype, vartype, sense] = rmt_constraints_PN_obs(Pre, Post, PN_marking, ...
                    data.T.props, Obs, 'final', obs_fin, obs_feasible, data.optim.param.intMarkings, ...
                    data.optim.param.alpha, data.optim.param.beta, data.optim.param.gamma, 'glpk'); %constraints for ILP for PN
                tic;
                [xmin, fmin, status, extra] = glpk(cost, matrix_A, vector_b, lb, ub, ctype, vartype, sense,data.optim.options_glpk);   %optimization with glpk
        
            case 'intlinprog'
                [cost, intcon, A, b, Aeq, beq, lb, ub] = rmt_constraints_PN_obs(Pre, Post, PN_marking, ...
                    data.T.props, Obs, 'final', obs_fin, obs_feasible, data.optim.param.intMarkings, ...
                    data.optim.param.alpha, data.optim.param.beta, data.optim.param.gamma, 'intlinprog'); %constraints for ILP for PN
                A_sparse=sparse(A); %can use sparse for large matrices
                Aeq_sparse=sparse(Aeq);
                tic;
                [xmin, fmin, status, extra] = intlinprog(cost, intcon, A_sparse, b, Aeq_sparse, beq, lb, ub, data.optim.options_milp);   %optimization with intlinprog

            case 'cplex'
                [cost, A, b, Aeq, beq, lb, ub, vartype]  = rmt_constraints_PN_obs(Pre, Post, PN_marking, ...
                    data.T.props, Obs, 'final', obs_fin, obs_feasible, data.optim.param.intMarkings, ...
                    data.optim.param.alpha, data.optim.param.beta, data.optim.param.gamma, 'cplex'); %constraints for ILP for PN
                A_sparse=sparse(A); %can use sparse for large matrices            Pre, Post, m0,          props,       Obs, obs_type, set_ind_fin, set_ind_traj, k, alpha, beta, gamma, solver
                Aeq_sparse=sparse(Aeq);
                tic;
                [xmin, fmin, status, extra] = cplexmilp(cost, A, b, Aeq, beq, [],[],[], lb, ub, ...
                    vartype, [], []);   %optimization with cplexmilp
        end
        tiempo = toc;
        total_time = total_time + tiempo;
        fprintf('\n\t ILP 1 finished in %g seconds: \n ',tiempo);
        
        prec=0.5e-5;    %additional tests for solution feasibility
        switch solver
            case 'glpk'
                if (status == 5) && (max(matrix_A*xmin-vector_b)>prec || max(lb-xmin)>prec) %this happened for too large M in constraints_PN_obs, for x_i and x_i(t) variables
                    fprintf('\nWrong solution returned by GLPK -> It doesn''t satisfy constraints! Correctness of results is not guaranteed.\n') 
                end
            case 'intlinprog' %this didn't occur; at least an error signaled by Matlab in intlinprog, caused actually by too large M in constraints_PN_obs, for x_i and x_i(t) variables
                if (status == 1) && (max(A*xmin-b)>prec || max(abs(Aeq*xmin-beq))>prec || max(lb-xmin)>prec || max(xmin-ub)>prec)
                    fprintf('\nWrong solution returned by INTLINPROG -> It doesn''t satisfy constraints! Correctness of results is not guaranteed.\n') 
                end

            case 'cplex' %this didn't occur
                if (status >= 0) && (max(A*xmin-b)>prec || max(abs(Aeq*xmin-beq))>prec || max(lb-xmin)>prec)
                    fprintf('\nWrong solution returned by CPLEX -> It doesn''t satisfy constraints! Correctness of results is not guaranteed.\n') 
                end
        end

        %test solution feasibility (non-spurious sigma and generated observables do not leave current transition in Buchi)
        [feasible_sol , marking_final, Rob_positions_final, Rob_places_temp, Rob_trans_temp, ...
            Rob_synchronize_temp, min_k1] = rmt_test_solution_feasibility(Pre, Post, PN_marking, ...
            data.T.props, Obs, 'final', obs_fin, obs_feasible, data.optim.param.intMarkings, solver, xmin, ...
            status, Rob_positions);
        
        if feasible_sol==1 %ILP1 gives good result, go to next transition in Buchi
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
        fprintf('\n\t Solving ILP 2 with solver %s ... ',solver);
        switch solver
            case 'glpk'
                [cost, matrix_A, vector_b, lb, ub, ctype, vartype, sense] = rmt_constraints_PN_obs(Pre, Post, PN_marking, ...
                    data.T.props, Obs, 'trajectory', obs_fin, obs_feasible, data.optim.param.intMarkings, ...
                    data.optim.param.alpha, data.optim.param.beta, data.optim.param.gamma, 'glpk'); %constraints for ILP for PN
                tic;
                [xmin, fmin, status, extra] = glpk(cost, matrix_A, vector_b, lb, ub, ctype, vartype, sense,data.optim.options_glpk);   %optimization with GLPK
            case 'intlinprog'
                [cost, intcon, A, b, Aeq, beq, lb, ub] = rmt_constraints_PN_obs(Pre, Post, PN_marking, ...
                    data.T.props, Obs, 'trajectory', obs_fin, obs_feasible, data.optim.param.intMarkings, ...
                    data.optim.param.alpha, data.optim.param.beta, data.optim.param.gamma, 'intlinprog'); %constraints for ILP for PN
                A_sparse=sparse(A); %can use sparse for large matrices
                Aeq_sparse=sparse(Aeq);
                tic;
                [xmin, fmin, status, extra] = intlinprog(cost, intcon, A_sparse, b, Aeq_sparse, beq, lb, ub, data.optim.options_milp);   %optimization with intlinprog
                
            case 'cplex'
                [cost, A, b, Aeq, beq, lb, ub, vartype]  = rmt_constraints_PN_obs(Pre, Post, PN_marking, ...
                    data.T.props, Obs, 'trajectory', obs_fin, obs_feasible, data.optim.param.intMarkings,...
                    data.optim.param.alpha, data.optim.param.beta, data.optim.param.gamma, 'cplex'); %constraints for ILP for PN
                A_sparse=sparse(A); %can use sparse for large matrices
                Aeq_sparse=sparse(Aeq);
                tic;
                [xmin, fmin, status, extra] = cplexmilp(cost, A, b, Aeq, beq, [],[],[], lb, ub, vartype, [], []);   %optimization with cplexmilp
        end
        tiempo = toc;
        total_time = total_time + tiempo;
        fprintf('\n\t ILP 2 finished in %g seconds: \n ',tiempo);

        prec=0.5e-5;    %additional tests for solution feasibility
        switch solver
            case 'glpk'
                if (status == 5) && (max(matrix_A*xmin-vector_b)>prec || max(lb-xmin)>prec) %this happened for too large M in constraints_PN_obs, for x_i and x_i(t) variables
                    fprintf('\nWrong solution returned by GLPK -> It doesn''t satisfy constraints! Correctness of results is not guaranteed.\n') 
                end
            case 'intlinprog' %this didn't occur; at least an error signaled by Matlab in intlinprog, caused actually by too large M in constraints_PN_obs, for x_i and x_i(t) variables
                if (status == 1) && (max(A*xmin-b)>prec || max(abs(Aeq*xmin-beq))>prec || max(lb-xmin)>prec || max(xmin-ub)>prec)
                    fprintf('\nWrong solution returned by INTLINPROG -> It doesn''t satisfy constraints! Correctness of results is not guaranteed.\n') 
                end

            case 'cplex' %this didn't occur
                if (status >= 0) && (max(A*xmin-b)>prec || max(abs(Aeq*xmin-beq))>prec || max(lb-xmin)>prec)
                    fprintf('\nWrong solution returned by CPLEX -> It doesn''t satisfy constraints! Correctness of results is not guaranteed.\n') 
                end
        end

        %test solution feasibility (non-spurious sigma and generated observables do not leave current transition in Buchi)
        [feasible_sol , marking_final, Rob_positions_final, Rob_places_temp, Rob_trans_temp, ...
            Rob_synchronize_temp, min_k2] = rmt_test_solution_feasibility(Pre, Post, PN_marking, ...
            data.T.props, Obs, 'trajectory', obs_fin, obs_feasible, data.optim.param.intMarkings, ...
            solver, xmin, status, Rob_positions);
        
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
        fprintf('\nIMPOSSIBLE to take current transition (s%g -> s%g) in Buchi automaton!\n',path_B(i),path_B(i+1));
        feasible_path_B=0; %current path in Buchi is not feasible
    end
    
    if feasible_path_B==1   %path in B was followed
        fprintf('\nSOUTION FOUND!\nTotal time of solving all MILPS: %d seconds\n',total_time)
        break;
    end
end

if feasible_path_B==0   %no path in B could be followed
    fprintf('\nNO SOLUTION! No path in B could be followed. \n\tPossible reasons:\n\t\t-the problem is unfeasible (impossible to satisfy task);\n\t\t-too few paths in Buchi;\n\t\t-too small number of intermediate PN markings.\n');
    fprintf('\nTotal time of solving all MILPS: %d seconds\n',total_time);
else %plot robot trajectories
    [rob_traj,synch_points] = rmt_rob_cont_traj_synch(data.T,Rob_places,Rob_synchronize,data.initial);    %continuous trajectory of each robot 
    rob_color={'r','b','g','c','m','k','y'};    %colors of robots
    data.trajectory = rob_traj;
    
    for r=1:N_r    %plot trajectories of robots
        plot(rob_traj{r}(1,1),rob_traj{r}(2,1),rob_color{mod(r-1,length(rob_color))+1},'Marker','o','LineWidth',1.5);
        plot(rob_traj{r}(1,:),rob_traj{r}(2,:),rob_color{mod(r-1,length(rob_color))+1},'LineWidth',1.5);
        plot(rob_traj{r}(1,end),rob_traj{r}(2,end),rob_color{mod(r-1,length(rob_color))+1},'Marker','x','LineWidth',1.5);
        plot(synch_points{r}(1,:),synch_points{r}(2,:),'k','Marker','s','MarkerSize',5);
    end
end
