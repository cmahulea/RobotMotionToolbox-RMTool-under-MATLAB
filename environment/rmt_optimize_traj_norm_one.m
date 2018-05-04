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


function [short_traj , traj_length] = rmt_optimize_traj_norm_one(com_F,path_cells,start_p,goal_p,safe_dist)
%% formulate LP for minimizing sum of norm 1 from all trajectory segments

n_lam=length(path_cells)-1; %number of segments crossed (lambda's to find)
short_traj=zeros(2,n_lam+2); %start, n_lam interm points, stop
X=cell(1,n_lam);
Y=cell(1,n_lam);
for i=1:n_lam  %boundary points of common facets (line segments)
    X{i}=com_F{path_cells(i),path_cells(i+1)}(1,:);    %x_i' and x_i'' (row)
    Y{i}=com_F{path_cells(i),path_cells(i+1)}(2,:);
end

%3*n_lam+2 variables in LP
% A=zeros(4*n_lam+4 , 3*n_lam+2);
% b=zeros(4*n_lam+4,1);
A=[];
b=[];
cost_f=[zeros(n_lam,1) ; ones(2*n_lam+2,1)];  %linear term of cost funct.
if n_lam==0 %particular cases
    short_traj=[start_p , goal_p];  %in the same cell
    return;
elseif n_lam==1
    ax1=zeros(1,3*n_lam+2); %first segment
    ax1(1)= (X{1}(2)-X{1}(1));
    ax2=-ax1;
    ax1(n_lam+1)=-1;
    ax2(n_lam+1)=-1;
    bx1= start_p(1)-X{1}(1);
    bx2= -bx1;
    ay1=zeros(1,3*n_lam+2); %current line in A (for y)
    ay1(1)= (Y{1}(2)-Y{1}(1));
    ay2=-ay1;
    ay1(2*n_lam+2)=-1;
    ay2(2*n_lam+2)=-1;
    by1= start_p(2)-Y{1}(1);
    by2= -by1;
    A=[A; ax1;ax2;ay1;ay2];
    b=[b; bx1;bx2;by1;by2];

    ax1=zeros(1,3*n_lam+2); %last segment
    ax1(n_lam)= -(X{n_lam}(2)-X{n_lam}(1));
    ax2=-ax1;
    ax1(2*n_lam+1)=-1;
    ax2(2*n_lam+1)=-1;
    bx1= X{n_lam}(1)-goal_p(1);
    bx2= -bx1;
    ay1=zeros(1,3*n_lam+2); %current line in A
    ay1(n_lam)= -(Y{n_lam}(2)-Y{n_lam}(1));
    ay2=-ay1;
    ay1(3*n_lam+2)=-1;
    ay2(3*n_lam+2)=-1;
    by1= Y{n_lam}(1)-goal_p(2);
    by2= -by1;
    A=[A; ax1;ax2;ay1;ay2];
    b=[b; bx1;bx2;by1;by2];
    
else %n_lam>=2 - formulas computed on paper
    ax1=zeros(1,3*n_lam+2); %first restrictions
    ax1(1)= (X{1}(2)-X{1}(1));
    ax2=-ax1;
    ax1(n_lam+1)=-1;
    ax2(n_lam+1)=-1;
    bx1= start_p(1)-X{1}(1);
    bx2= -bx1;
    
    ay1=zeros(1,3*n_lam+2);
    ay1(1)= (Y{1}(2)-Y{1}(1));
    ay2=-ay1;
    ay1(2*n_lam+2)=-1;
    ay2(2*n_lam+2)=-1;
    by1= start_p(2)-Y{1}(1);
    by2= -by1;
    
    A=[A; ax1;ax2;ay1;ay2];
    b=[b; bx1;bx2;by1;by2];
    
    for i=1:(n_lam-1)
        ax1=zeros(1,3*n_lam+2); %current line in A (for X)
        ax1(i)= -(X{i}(2)-X{i}(1));
        ax1(i+1)= (X{i+1}(2)-X{i+1}(1));
        ax2=-ax1;
        ax1(n_lam+1+i)=-1;
        ax2(n_lam+1+i)=-1;
        bx1= X{i}(1)-X{i+1}(1);
        bx2= -bx1;
        
        ay1=zeros(1,3*n_lam+2); %current line in A (for Y)
        ay1(i)= -(Y{i}(2)-Y{i}(1));
        ay1(i+1)= (Y{i+1}(2)-Y{i+1}(1));
        ay2=-ay1;
        ay1(2*n_lam+2+i)=-1;
        ay2(2*n_lam+2+i)=-1;
        by1= Y{i}(1)-Y{i+1}(1);
        by2= -by1;
        
        A=[A; ax1;ax2;ay1;ay2];
        b=[b; bx1;bx2;by1;by2];
    end
    
    ax1=zeros(1,3*n_lam+2); %constraints for last segment
    ax1(n_lam)= -(X{n_lam}(2)-X{n_lam}(1));
    ax2=-ax1;
    ax1(2*n_lam+1)=-1;
    ax2(2*n_lam+1)=-1;
    bx1= X{n_lam}(1)-goal_p(1);
    bx2= -bx1;
    
    ay1=zeros(1,3*n_lam+2);
    ay1(n_lam)= -(Y{n_lam}(2)-Y{n_lam}(1));
    ay2=-ay1;
    ay1(3*n_lam+2)=-1;
    ay2(3*n_lam+2)=-1;
    by1= Y{n_lam}(1)-goal_p(2);
    by2= -by1;
    
    A=[A; ax1;ax2;ay1;ay2];
    b=[b; bx1;bx2;by1;by2];
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
low_bound=[zeros(n_lam,1)+epsilon ; zeros(2*n_lam+2,1)];  %zero lower limit for abs. values (c's)
up_bound=ones(n_lam,1)-epsilon;
A=[A; [eye(n_lam) , zeros(n_lam,2*n_lam+2)]]; %upperbound for lamb. in A, because no upper for c
b=[b; up_bound];

%%LP optimization
try
    %use Matlab's Optimization Toolbox:
    options_lp=optimoptions(@linprog,'Display','off','Algorithm','simplex');
    [sol,~,exitflag] = linprog(cost_f,A,b,[],[],low_bound,[],[],options_lp); %solve LPP
    if exitflag~=1
        fprintf('\nERROR when solving LP for norm one.\n');
    end
catch
    %use LP solver from GLPK
    ctype=''; %type of constraints
    for i = 1 : size(A,1)
        ctype = sprintf('%sU',ctype);   %inequality <=
    end
    vartype = ''; % type of variables
    for i = 1 : size(A,2)     %all variables are continuous (LPP)
        vartype = sprintf('%sC',vartype); %continuous
    end
    [sol, ~, status] = glpk (cost_f, A, b, low_bound, [], ctype, vartype, 1); %minimization
    if status~=5
        fprintf('\nERROR when solving LP for norm one with GLPK.\n');
    end
end

lambda=sol(1:n_lam);    %no need for c's
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
