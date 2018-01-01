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
%   First version released on September, 2014. 
%   Last modification December 29, 2015.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function [short_traj , traj_length] = optimize_traj_norm_two_sq(com_F,path_cells,start_p,goal_p,safe_dist)
%% formulate QP for minimizing sum of squares of lengths from trajectory segments through given cells

n_lam=length(path_cells)-1; %number of segments crossed (lambda's to find)
short_traj=zeros(2,n_lam+2); %start, n_lam interm points, stop
X=cell(1,n_lam);
Y=cell(1,n_lam);
for i=1:n_lam  %boundary points of common facets (line segments)
    X{i}=com_F{path_cells(i),path_cells(i+1)}(1,:);    %x_i' and x_i'' (row)
    Y{i}=com_F{path_cells(i),path_cells(i+1)}(2,:);
end

cost_f=zeros(n_lam,1);  %linear term of cost funct
cost_H=zeros(n_lam,n_lam);  %quadr. term of cost funct
if n_lam==0 %particular cases
    short_traj=[start_p , goal_p];  %in the same cell
    return;
elseif n_lam==1
    cost_f(1)=2*( (X{1}(2)-X{1}(1)) * (2*X{1}(1)-start_p(1)-goal_p(1)) + (Y{1}(2)-Y{1}(1)) * (2*Y{1}(1)-start_p(2)-goal_p(2)) );
    cost_H(1,1) = 4*( (X{1}(2)-X{1}(1))^2 + (Y{1}(2)-Y{1}(1))^2 );
    
else %n_lam>=2 - formulas computed on paper
    cost_f(1) = 2*( (X{1}(2)-X{1}(1)) * (2*X{1}(1)-start_p(1)-X{2}(1)) + (Y{1}(2)-Y{1}(1)) * (2*Y{1}(1)-start_p(2)-Y{2}(1)) );
    cost_H(1,1) = 4*( (X{1}(2)-X{1}(1))^2 + (Y{1}(2)-Y{1}(1))^2 );  %optimize f'*lam + 1/2 *lam'*H*lam
    cost_H(1,2) = -2*( (X{1}(2)-X{1}(1))*(X{2}(2)-X{2}(1)) + (Y{1}(2)-Y{1}(1))*(Y{2}(2)-Y{2}(1)) );    %make symmetric matrix
    cost_H(2,1) = cost_H(1,2);
    for i=2:(n_lam-1)
        cost_f(i) = 2*( (X{i}(2)-X{i}(1)) * (2*X{i}(1)-X{i-1}(1)-X{i+1}(1)) + (Y{i}(2)-Y{i}(1)) * (2*Y{i}(1)-Y{i-1}(1)-Y{i+1}(1)) );
        cost_H(i,i) = 4*( (X{i}(2)-X{i}(1))^2 + (Y{i}(2)-Y{i}(1))^2 );
        cost_H(i,i+1) = -2*( (X{i}(2)-X{i}(1))*(X{i+1}(2)-X{i+1}(1)) + (Y{i}(2)-Y{i}(1))*(Y{i+1}(2)-Y{i+1}(1)) );  %for a symmetric matrix
        cost_H(i+1,i) = cost_H(i,i+1);
    end
    
    cost_f(n_lam) = 2*( (X{n_lam}(2)-X{n_lam}(1)) * (2*X{n_lam}(1)-X{n_lam-1}(1)-goal_p(1)) + (Y{n_lam}(2)-Y{n_lam}(1)) * (2*Y{n_lam}(1)-Y{n_lam-1}(1)-goal_p(2)) );
    cost_H(n_lam,n_lam) = 4*( (X{n_lam}(2)-X{n_lam}(1))^2 + (Y{n_lam}(2)-Y{n_lam}(1))^2 );
end

positivedefinite = all(eig(cost_H) > 0);    %positive definite matrix (or function chol)
if positivedefinite ~= 1
    fprintf('\n WARNING: Quadratic cost matrix is not positive definite => non-convex cost fcn. => suboptimal solution may be obtained.\n')
end
%safe distances transformed to limits of lambda (for each segment)
epsilon=zeros(n_lam,1);
for i=1:n_lam
    len=norm([X{i}(2)-X{i}(1) ; Y{i}(2)-Y{i}(1)]);   %length of current segment
    epsilon(i)=safe_dist/len;
    if epsilon(i)>0.5   %test for safe_dist
        fprintf('\n ERROR: Safe distance is larger than half of minimum length crossed segment!\n');
        return
    end
end
low_bound=zeros(n_lam,1)+epsilon;
up_bound=ones(n_lam,1)-epsilon;
%form with bounds in matrix A and vector b is more appropriate (otherwise warnings or errors may result)
A=[-eye(n_lam); eye(n_lam)];
b=[-low_bound ; up_bound];

%%QP optimization
try
    %use Matlab's Optimization Toolbox:
    % options_qp=optimoptions(@quadprog,'Display','off','Algorithm','interior-point-convex');    %for convex problem
    options_qp=optimoptions(@quadprog,'Display','off','Algorithm','active-set');    %cost seems better (lambdas closer to edges
    [lambda,~,exitflag] = quadprog(cost_H,cost_f,A,b,[],[],[],[],[],options_qp); %in A and b: only lower/upper bound constraints
    if exitflag~=1
        fprintf('\nERROR when solving QP for sum of sqyares of norm two.\n');
    end
catch
    %use QP solver from GLPK (QNPG)
    ctype=''; %type of constraints
    for i = 1 : size(A,1)
        ctype = sprintf('%sU',ctype);   %inequality <=
    end
    [lambda,~,~,info] = qpng(cost_H, cost_f, A, b, ctype, [], [], 0.5*ones(n_lam,1)); %QP, initial sol middle points; otherwise it seems that GLPK's QP may sometimes violate constraints
    if info.status~=1
        fprintf('\nERROR when solving QP for sum of sqyares of norm two (with GLPK).\n');
    end
end

%hopefully shorter traj
short_traj(:,1)=start_p;
for i=1:n_lam
    short_traj(:,i+1)=(1-lambda(i))*[X{i}(1);Y{i}(1)] + lambda(i)*[X{i}(2);Y{i}(2)];
end
short_traj(:,end)=goal_p;

%travelled distance (Euclidean norm)
traj_length=0;
for i=1:(size(short_traj,2)-1)
    traj_length = traj_length + norm(short_traj(:,i+1)-short_traj(:,i));
end
