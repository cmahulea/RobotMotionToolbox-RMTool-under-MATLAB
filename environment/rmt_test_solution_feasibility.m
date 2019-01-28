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
%   First version released on January, 2019. 
%   Last modification January 31, 2019.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function [feasible_sol , marking_final, Rob_positions_final, Rob_places, Rob_trans, Rob_synchronize, tr_number] = rmt_test_solution_feasibility(Pre, Post, m0, props, Obs, obs_type, set_ind_fin, set_ind_traj, k, solver, xmin, status, Rob_positions)
%if optimizxation on PN model finished, test feasibility of obtained solution
%this is done in two steps:
%1. sigma should be feasible (non-spurious)
%2. (ony for ILP1:) if 1. is true, all possible generated team observations during unsynchronized robot movements should be included in a desired set (that does not leave desired traj. in Buchi)
    %the movement are unsynchronized until entering the final marking -> the moving robots synchronize when ENTERING the final marking, NOT INSIDE the final marking (otherwise formulas with synchronous movements couldn't become true)
%"xmin" is the solution returned by ILP solver, "status" is the optimization status
%Rob_positions is the vector with current initial robot positions (for constructing their runs, when feasible solution is obtained)
%the other input arguments are as in function "constraints_PN_obs"

%outputs:
%feasible_sol is 1 if trajectory is feasible, and zero otherwise
%if feasible_sol is 1, the other outputs have the following meaning:
%marking_final is the PN model final marking
%Rob_positions_final is the vector with final robots' positions (in correspondence with marking_final)
%Rob_places{i} is a vector containing sequence of places through which robot i travelled (in order in which they appear)
%Rob_trans{i} is a vector containing sequence of transitions which robot i followed (in order in which they fire)
%Rob_synchronize{i} contains indices of visited cells when robot i synchronizes with others (intermediate and final marking)
%synchronization is done when entering the corresponding region (except of course the initial state, and the non-moving robots which just wait for others to enter their cells)
%tr_number is the number of transitions that fired; if optimization did not succeeded, is zero; it is useful for finding minimum value of k for ILP3, when first running this function for ILPs 1 and 2

%%initialization
nplaces = size(Pre,1); %number of places
ntrans = size(Pre,2); %number of transitions
N_r = sum(m0); % number of robots
N_p = length(props); %number of atomic props.

if ~strcmp(obs_type,'intermediate')
    k=1;    %for 'final' or 'trajectory'
end

set_ind_traj = union(set_ind_traj,set_ind_fin); %set_ind_traj should include set_ind_fin (see explanation lines at beginning)


%%initialize return arguments:
feasible_sol=0;   %assume path in PN is not good (reaches observations that leave path in Buchi)
marking_final = [];
Rob_positions_final = [];
Rob_places = cell(1,N_r);
Rob_trans = cell(1,N_r);
Rob_synchronize = cell(1,N_r);

for j=1:N_r
    Rob_places{j}=Rob_positions(j); %initial position
    Rob_trans{j}=[];
    Rob_synchronize{j}=[];
end
tr_number=0;

%%test optimization status
% prec=eps*1e5;   %precision for deciding integers
prec=1e-5;
switch solver
    case 'glpk'
        if (status ~= 5) || (max(abs(xmin-round(xmin)))>prec)    %optimization did not finish, or solution is not close to integer values
            fprintf('\nGLPK optimization did not successfully finish (no solution found)!\n')
            return; %return function, with feasible_traj on zero
        end
    case 'intlinprog'
        if (status ~= 1) || (max(abs(xmin-round(xmin)))>prec)    %optimization did not finish, or solution is not close to integer values
            fprintf('\nINTLINPROG optimization did not successfully finish (no solution found)!\n')
            return; %return function, with feasible_traj on zero
        end
    case 'cplex'
        if (status < 0) || (max(abs(xmin-round(xmin)))>prec)    %optimization did not finish, or solution is not close to integer values
            fprintf('\nCPLEX optimization did not successfully finish (no solution found)!\n')
            return; %return function, with feasible_traj on zero
        end
end
xmin=round(xmin);   %in case small error appear due to interior point method


%%test 1: test for non-spurious sigma and if true, construct robot trajectories and transitions:
%for k>1 ('intermediate'), sigma_i's are non-spurious and observables are good, but we use "sigma2trajectories" function to construct robot trajectories

for i=1:k
    sigma = xmin((i-1)*(nplaces+ntrans)+nplaces+1 : i*(nplaces+ntrans)); %solution sigma_i
    
    [feasible_sigma, Rob_places_i, Rob_trans_i, Rob_positions_next] = ...
        rmt_sigma2trajectories(Pre,Post,m0,sigma,Rob_positions); %construct places and transition sequences in PN
    if feasible_sigma==0 %spurious sigma
        fprintf('\nSpurious sigma obtained! ILP constraints type: %s.\n',obs_type)
        return;
    end
    %for feasible sigma, update:
    Rob_positions = Rob_positions_next;
    m0 = xmin((i-1)*(nplaces+ntrans)+1 : (i-1)*(nplaces+ntrans)+nplaces); %update initial marking with marking_{i-1} for i^th iteration
    if sum(sigma)>0 %at least one transition fires
        for j=1:N_r
            Rob_places_i{j}(1)=[];  %initial position already included
            Rob_places{j} = [Rob_places{j} , Rob_places_i{j}];
            Rob_trans{j} = [Rob_trans{j} , Rob_trans_i{j}];
            Rob_synchronize{j} = [Rob_synchronize{j} , length(Rob_places{j})];  %synchronize in the actual place from trajectory of robot j (intermediary or final marking)
            tr_number = tr_number+length(Rob_trans_i{j});  %add the number of fired transitions
        end
    end
end

marking_final = xmin((k-1)*(nplaces+ntrans)+1 : (k-1)*(nplaces+ntrans)+nplaces); %final PN marking from solution of ILP1,ILP2,ILP3
Rob_positions_final = Rob_positions; %if for loop finished


% %%test 2: for non-spurious sigma, construct generated team observables to test if PN markings satisfy observations for path_B(i) -> path_B(i+1) (with possible loops in path_B(i))
% %test 2 may be more time consuming (it has a cartesian product);
% %for ILP3, test1 and test2 are not needed (they are true); test1 was run to get robot trajectories
% %test2 is not run for ILP3, since it's anyway true
% 
% if ~strcmp(obs_type,'intermediate') %ILP1 or ILP2
%     team_observ = traj2observ(Rob_places , props , Obs); %construct possible team observations resulting from unsynchronized robot movements
%     if ~isempty(setdiff(team_observ , set_ind_traj)) %it is possible to generate undesired observations -> not feasible solution; set_ind_traj includes set_ind_fin
%         fprintf('\nUndesired team observations can be generated! ILP constraints type: %s.\n',obs_type)
%         return;
%     end
% end

%%test 2: for non-spurious sigma, construct generated team observables to test if PN markings satisfy observations for path_B(i) -> path_B(i+1) (with possible loops in path_B(i))
%test 2 may be more time consuming (it has a cartesian product);
%for ILP3, test1 and test2 are not needed (they are true); test1 was run to get robot trajectories
%test2 is run nly for ILP1

if strcmp(obs_type,'final') %ILP1
    team_observ = rmt_traj2observ(Rob_places , props , Obs); %construct possible team observations resulting from unsynchronized robot movements
    if ~isempty(setdiff(team_observ , set_ind_traj)) %it is possible to generate undesired observations -> not feasible solution; set_ind_traj includes set_ind_fin
        fprintf('\tUndesired team observations can be generated! ILP constraints type: %s.\n',obs_type)
        return;
    end
end


feasible_sol = 1; %if this point is reached, solution is feasible
fprintf('\tCurrent solution is feasible!\n')
