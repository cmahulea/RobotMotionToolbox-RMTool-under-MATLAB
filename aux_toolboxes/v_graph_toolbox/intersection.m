%    This is part of RMTool - Robot Motion Toolbox, for visibility graph functionality.
%
%    Copyright (C) 2016 RMTool developing team, with support of Silvia Magdici. For people, details and citing 
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

function [existFlag dist X] = intersection( point1, point2, V_repres )


%%convert between V- and H- representations of a polytope
%from V- to H- :
%V_repres = vertices on columns
H=cddmex('hull',struct('V',V_repres')); %H-representation
A=H.A;
b=H.B;  %H_repres of polytope: A_pol * x + b_pol <= 0;

%%solve LP
[Aeq beq lb ub] = makeLine(point1, point2);
beq=-beq;
%epsilon = eps*1e10;
%lb= lb+epsilon;
%ub=ub-epsilon;

%c=[1;1];
c=(point2-point1)';
%========== with Matlab's LP solver==========
%restrictions: A*X <= b ; Aeq*X = beq ; lb <= X <= ub
% disp('Matlab LP solver linprog')
% options = optimset('LargeScale', 'off', 'Simplex', 'on', 'Display', 'off');   %use simplex method for LP
% [X,optim_cost,exitflag] = linprog(c,A,b,Aeq,beq,lb,ub,[],options);
% if exitflag~=1  %LP result: optimization didn't converged to a feasible solution
%     disp('No intersection.')
% else
%     disp('Intersection exists:')
%     disp(X)
% end
% existFlag = exitflag;
% %========== with CDD's LP solver==========
% set restrictions in form: A_cdd*X <= b_cdd for rows not specified in indices IN.lin; for indices in IN.lin: A_cdd(IN.lin,:)*X = b_cdd(IN.lin)
disp('CDD LP solver')
A_cdd=[Aeq ; A ; -eye(2) ; eye(2)]; %restrictions in order: equalities from Aeq; inequalities from A; inequalities for X>=lb; inequalities for X<=ub
b_cdd=[beq ; b ; -lb ; ub];
%for running CDD:
IN=struct('obj',c','A',A_cdd,'B',b_cdd,'lin',1:size(Aeq,1));
OUT=cddmex('solve_lp',IN) ;
X=OUT.xopt;
if OUT.how~=1  %LP: optimization didn't converged to a feasible solution
    disp('No intersection.')
    dist=0;
else
    disp('Intersection exists:')
    disp(X)
    disp(norm(X-point2'))
    dist=norm(X-point2');
end
existFlag= OUT.how;
end

