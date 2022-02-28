
%    This is part of RMTool - Robot Motion Toolbox, for Matlab 2010b or newer.
%
%    Copyright (C) 2016 RMTool developing team. For people, details and citing
%    information, please see: http://webdiis.unizar.es/RMTool/index.html.
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

function [xmin, message] = rmt_path_planning_pn_bool_new_traj(data,m0,mf,message)
%Path-planning with LTL specificaion and Petri net models, recomputing the robot trajectories by allocating
%the common resources in an optimal way (first in, first served)
%input: data - extract info about matrices Pre, Post, and solver
%       m0 - initial marking
%       mf  - final marking
%       message - update message with info about run time and number of
%       variables
% output: xmin - solution to MILP for re-planning trajectories 

Pre = data.relres.Pre;
Post = data.relres.Post;

nplaces = size(Pre,1); %number of places
ntrans = size(Pre,2); % number of transitions
No_r = length(data.RO);

C = Post - Pre;

%the variables are: [m_0^1 m_1 sigma_1 m_0^2 m_2 sigma_2 ... m_0^|R| m_|R| sigma_|R|]
Aeq = [];
beq = [];
A = [];
b = [];

temp_zeros = zeros(nplaces,2*nplaces + ntrans);
temp_mi = [zeros(nplaces) eye(nplaces) zeros(nplaces,ntrans)]; % is considered only the variable m_i
temp_m0 = [eye(nplaces) zeros(nplaces) zeros(nplaces, ntrans)]; % is considered only the variable m0^i
temp_m0_single = [ones(1,nplaces) zeros(1,nplaces), zeros(1,ntrans)];
temp_mi_single = [ zeros(1,nplaces) ones(1,nplaces) zeros(1,ntrans)];

tic
for i = 1:No_r

    % a) state equation
    Aeq = [Aeq; repmat(temp_zeros,[1, i-1]) eye(nplaces) -eye(nplaces) C repmat(temp_zeros,[1, No_r - i])];
    beq = [beq; zeros(nplaces,1)];

    % b) The robot i considers that i-1 robots arrived in the final marking
    % and the next i+1 robots are still in the  initial marking
    if i < No_r
        A = [A; repmat(temp_mi,[1,i-1]) zeros(nplaces,2*nplaces) Post repmat(temp_m0,[1, No_r - i])];
        b = [b; ones(nplaces,1)];
    else
        A = [A; repmat(temp_mi,[1,i-1]) zeros(nplaces,2*nplaces) Post];
        b = [b; ones(nplaces,1)];
    end

    % e) assures that only one robot moves
    Aeq = [Aeq; repmat(zeros(1,2*nplaces+ntrans),[1, i-1]) temp_m0_single repmat(zeros(1,2*nplaces+ntrans),[1, No_r - i])];
    beq = [beq; ones(1,1)];

    % f) assures that only one robot moves - possible redundant
    Aeq = [Aeq; repmat(zeros(1,2*nplaces+ntrans),[1, i-1]) temp_mi_single repmat(zeros(1,2*nplaces+ntrans),[1, No_r - i])];
    beq = [beq; ones(1,1)];

end

% c) all robots depart from the initial marking
Aeq = [Aeq; repmat(temp_m0, [1,No_r])];
beq = [beq; m0];

% d) all robots arrives in the final marking
Aeq = [Aeq; repmat(temp_mi, [1,No_r])];
beq = [beq; mf];

cost = [];
for i = 1:No_r
    cost = [cost zeros(1,2*nplaces) i*ones(1,ntrans)];
end
time = toc;
message = sprintf('\n %s Run time to construct the MILP is: %d , with %d number of variables ', message, time, No_r * (2*nplaces + ntrans));

data = get(gcf,'UserData');
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

switch solver
    case 'cplex'
        message = sprintf('%s\n\nThe MILP solution is with CPLEX\n\n', message);
        vartype = '';
        for r = 1:No_r

            for j = 1 : nplaces
                vartype = sprintf('%sI',vartype); %put the initial markings as integer
            end
            
             for j = 1 : nplaces
                vartype = sprintf('%sC',vartype); %put the final markings as real
            end
            for j = 1 : ntrans
                vartype = sprintf('%sI',vartype); %put the sigma as integer
            end
        end

        tic
        [xmin,fmin,exitflag] = cplexmilp(cost,A,b,Aeq,beq,[],[],[],zeros(1,size(A,2)),[],vartype);
        time = toc;
        switch exitflag
            case 6
                uiwait(errordlg('Non-optimal Solution available.','Robot Motion Toolbox','modal'));
            case 5
                uiwait(errordlg('Solution with numerical issues.','Robot Motion Toolbox','modal'));
                %case 1,
                %uiwait(errordlg('Function converged to a solution x.','Robot Motion Toolbox','modal'));
            case 0
                uiwait(errordlg('Number of iterations exceeded options.MaxIter.','Robot Motion Toolbox','modal'));
            case -1
                uiwait(errordlg('Aborted.','Robot Motion Toolbox','modal'));
            case -2
                uiwait(errordlg('No feasible point was found.','Robot Motion Toolbox','modal'));
            case -3
                uiwait(errordlg('Problem is unbounded.','Robot Motion Toolbox','modal'));
            case -4
                uiwait(errordlg('NaN value was encountered during execution of the algorithm.','Robot Motion Toolbox','modal'));
            case -5
                uiwait(errordlg('Both primal and dual problems are infeasible.','Robot Motion Toolbox','modal'));
            case -7
                uiwait(errordlg('Search direction became too small. No further progress could be made.','Robot Motion Toolbox','modal'));
            case -8
                uiwait(errordlg('Problem is infeasible or unbounded.','Robot Motion Toolbox','modal'));
            case -9
                uiwait(errordlg('Limit reached.','Robot Motion Toolbox','modal'));
        end
        message = sprintf('\n %s Run time to solve the MILP is: ',message, time);
    case 'glpk'
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
        for i = 1 : No_r
            for j = 1 : nplaces
                vartype = sprintf('%sI',vartype); %put the initial markings as integer
            end
            
             for j = 1 : nplaces
                vartype = sprintf('%sC',vartype); %put the final markings as real
            end
            for j = 1 : ntrans
                vartype = sprintf('%sI',vartype); %put the sigma as integer
            end
        end

        Atot = [Aeq; A];
        btot= [beq; b];
        ctype = [ctype2 ctype1];
        tic
        [xmin,fmin,~] = glpk(cost,Atot,btot,zeros(1,size(A,2)),[],ctype,vartype);
        time = toc;
        message = sprintf('\n %s Run time to solve the MILP is: ',message, time);

    case 'intlinprog'
        message = sprintf('%s\n\nThe MILP solution is with intlinprog\n\n', message);
        tic
        [xmin,fmin,~] = intlinprog(cost, 1:length(cost), A, b, Aeq, beq, zeros(1,length(cost)), []);
        toc
        message = sprintf('\n %s Run time to solve the MILP is: ',message, time);

end
message = sprintf('\n %s Cost function is: %d ',message, fmin);

if (isempty(xmin)) %no solution
    uiwait(errordlg('Error solving the ILP. The problem may have no feasible solution!','Robot Motion Toolbox','modal'));
    return;
end

% new trajectories of the robots and the new order

end
