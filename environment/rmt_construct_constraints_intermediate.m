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

function [A,b,Aeq,beq,cost,obstacles] = rmt_construct_constraints_intermediate(Pre,Post,m0, props, formula)

%the variables are: [m_1 sigma_1 m_2 sigma_2 .... m_r+1(mi) sigma_r+1 no_of_binary_variables_traj ]

nplaces = size(Pre,1); %number of places
ntrans = size(Pre,2); % number of transitions
N = sum(m0); % number of robots

Aeq = [eye(nplaces) -(Post-Pre) zeros(nplaces,(nplaces+ntrans)*N)];
beq = m0;
for i = 2 : N+1
    %add the state equation: m_i = m_{i-1} + (Post-Pre)*sigma_i
    Aeq = [Aeq;zeros(nplaces,(nplaces+ntrans)*(i-2)) -eye(nplaces) zeros(nplaces,ntrans) eye(nplaces) -(Post-Pre) zeros(nplaces,(nplaces+ntrans)*(N+1-i))];
    beq = [beq; zeros(nplaces,1)];
end
A=[];
b=[];

%define the boolean variables for the final (intermediate) marking
for i = 1 : length(props)
    %define the vectors v
    vii = zeros(1,size(Aeq,2)); %intermediate state
    vii(props{i} + (nplaces+ntrans)*N)=1;
    % add the boolean variables xi
    Aeq = [Aeq zeros(size(Aeq,1),1)]; % add a new column to Aeq
    if isempty(A)
        A = zeros(0,size(Aeq,2));
    else
        A = [A zeros(size(A,1),1)]; %add a new columns to A
    end
    %add the constraints of the formula for intermediate state
    A = [A ; vii -N]; %sum(vii*m_i)-N*xi \leq 0
    b = [b ; 0];
    A = [A ; -vii 1]; %xi-sum(vii*m_i) \leq 0
    b = [b ; 0];
end

%add constraints for collision avoidance

A = [A; zeros(nplaces,nplaces) Post zeros(nplaces,size(Aeq,2)-(nplaces+ntrans))];
b = [b; ones(nplaces,1)-m0];
 
for i = 2 : N+1
    A = [A; zeros(nplaces,(nplaces+ntrans)*(i-2)) eye(nplaces) zeros(nplaces,ntrans+nplaces) Post zeros(nplaces,size(Aeq,2)-i*(nplaces+ntrans))];
    b = [b; ones(nplaces,1)];
end
%%%%%%%%%%% cost function
cost = zeros(1,size(Aeq,2));
for i = 1 : N+1
    cost((i-1)*(nplaces+ntrans)+nplaces+1:(i-1)*(nplaces+ntrans)+ntrans+nplaces) = i*10;
end
%parse formula and convert to linear constraints/restrictions
[~,~,~,A_r,b_r] = rmt_formula2constraints(formula, A,b,length(props));
A_r(:,length(props)+1:end) = [];
for i = size(A_r,1):-1:1  %remove the empty lines that corresppnds to formula on the final state
    if isempty(find(A_r(i,:)))
        A_r(i,:) = [];
        b_r(i) = [];
    end
end

obstacles=[];
for i = size(A_r,1):-1:1
    if ((length(find(A_r(i,:)))==1) && (b_r(i)==0))
        obstacles = union(obstacles,find(A_r(i,:)));
        A_r(i,:) = [];
        b_r(i) = [];
    end
end

if ~isempty(A_r)
    A = [A; zeros(size(A_r,1),(N+1)*(nplaces+ntrans)) A_r];
    b = [b;b_r];
end

if ~isempty(obstacles)
    eta = zeros(1,ntrans);
    for j = 1 : length(obstacles)
        cells = props{obstacles(j)};
        for k = 1 : length(cells)
            eta(find(Post(cells(k),:))) = 1;
        end
    end
    for j = 1 : N+1
        Aeq = [Aeq; zeros(1,(j-1)*(nplaces+ntrans)) zeros(1,nplaces) eta zeros(1,size(Aeq,2)-(j)*(nplaces+ntrans))];
        beq = [beq;0];
    end
end
