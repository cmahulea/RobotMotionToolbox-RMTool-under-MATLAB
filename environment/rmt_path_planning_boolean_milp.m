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
%   First version released on November, 2019. 
%   Last modification November 2019.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================
function [xmin,message] = rmt_path_planning_boolean_milp(cost,A,b,Aeq,beq,nplaces,ntrans,intermediate_markings, N_props, message)

data = get(gcf,'UserData');
if strcmp(get(data.optim.menuCplex,'Checked'),'on')
    message = sprintf('%s\n\nThe MILP solution is with CPLEX\n\n', message);
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    vartype = '';
    for i = 1 : intermediate_markings
        for j = 1 : nplaces
            vartype = sprintf('%sC',vartype); %put the markings as real
        end
        for j = 1 : ntrans
            vartype = sprintf('%sI',vartype); %put the sigma as integer
        end
    end
    for i = 1 : N_props
        vartype = sprintf('%sB',vartype); %put the binary variable
    end
    [xmin,~,exitflag] = cplexmilp(cost,A,b,Aeq,beq,[],[],[],zeros(1,size(A,2)),[],vartype);
    switch exitflag
        case 6,
            uiwait(errordlg('Non-optimal Solution available.','Robot Motion Toolbox','modal'));
        case 5,
            uiwait(errordlg('Solution with numerical issues.','Robot Motion Toolbox','modal'));
        %case 1,	
            %uiwait(errordlg('Function converged to a solution x.','Robot Motion Toolbox','modal'));
        case 0,
            uiwait(errordlg('Number of iterations exceeded options.MaxIter.','Robot Motion Toolbox','modal'));
        case -1,
            uiwait(errordlg('Aborted.','Robot Motion Toolbox','modal'));
        case -2,
            uiwait(errordlg('No feasible point was found.','Robot Motion Toolbox','modal'));
        case -3,
            uiwait(errordlg('Problem is unbounded.','Robot Motion Toolbox','modal'));
        case -4,
            uiwait(errordlg('NaN value was encountered during execution of the algorithm.','Robot Motion Toolbox','modal'));
        case -5,
            uiwait(errordlg('Both primal and dual problems are infeasible.','Robot Motion Toolbox','modal'));
        case -7,
            uiwait(errordlg('Search direction became too small. No further progress could be made.','Robot Motion Toolbox','modal'));
        case -8,
            uiwait(errordlg('Problem is infeasible or unbounded.','Robot Motion Toolbox','modal'));
        case -9,
            uiwait(errordlg('Limit reached.','Robot Motion Toolbox','modal'));
    end
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
    for i = 1 : intermediate_markings
        for j = 1 : nplaces
            vartype = sprintf('%sC',vartype); %put the markings as real
        end
        for j = 1 : ntrans
            vartype = sprintf('%sI',vartype); %put the sigma as integer
        end
    end
    for i = 1 : N_props
        vartype = sprintf('%sB',vartype); %put the binary variable
    end
    Atot = [Aeq; A];
    btot= [beq; b];
    ctype = [ctype2 ctype1];
    [xmin,~,~] = glpk(cost,Atot,btot,zeros(1,size(A,2)),[],ctype,vartype);
end
if (isempty(xmin)) %no solution
    uiwait(errordlg('Error solving the ILP. The problem may have no feasible solution!','Robot Motion Toolbox','modal'));
    return;
end