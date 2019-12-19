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

function [A,b,Aeq,beq,cost] = rmt_construct_constraints(Pre,Post,m0, props, k, formula)

%the variables are: [m_i sigma_i 2Xno_of_binary_variables b], i = 1,...,k

nplaces = size(Pre,1); %number of places
ntrans = size(Pre,2); % number of transitions

%add the state equation: m_{i+1} = m_i + (Post-Pre)*sigma_{i+1}

Aeq = [eye(nplaces) -(Post-Pre)];
beq = m0;
A = [zeros(nplaces,nplaces) Pre]; %m0 - Pre \cdot sigma \geq 0
b = m0;

for i = 2 : k
    Aeq = [Aeq zeros(size(Aeq,1),nplaces+ntrans)]; %add nplaces+ntrans columns to Aeq
    Aeq = [Aeq; zeros(nplaces,(i-2)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) eye(nplaces) -(Post-Pre)]; %state equation
    beq = [beq;zeros(nplaces,1)];
    
    A = [A zeros(size(A,1),nplaces+ntrans)]; %add nplaces+ntrans columns to A
    A = [A ; zeros(nplaces, (i-2)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) zeros(nplaces,nplaces) Pre];
    b = [b ; zeros(nplaces,1)];
end
N = sum(m0); % number of robots


%define the boolean variables for the trajectories
for i = 1 : length(props)
    %define the vectors v
    vi = zeros(1,k*(nplaces+ntrans));
    for j = 1 : k-1
        vi(props{i}+(j-1)*(nplaces+ntrans))=1;
        % add the boolean variables xi
    end
    Aeq = [Aeq zeros(size(Aeq,1),1)]; % add a new column to Aeq
    if isempty(A)
        A = zeros(0,size(Aeq,2));
    else
        A = [A zeros(size(A,1),1)]; %add a new column to A
    end
    %add the constraints of the formula
    A = [A ; vi zeros(1,size(Aeq,2)-length(vi)-1) -N]; %sum(vi*m_i)-N*xi \leq 0
    b = [b ; 0];
    A = [A ; -vi zeros(1,size(Aeq,2)-length(vi)-1) 1]; %xi-sum(vi*m_i) \leq 0
    b = [b ; 0];
end

%define the boolean variables for the final states
for i = 1 : length(props)
    %define the vectors v
    vi = zeros(1,nplaces);
    vi(props{i})=1;
    % add the boolean variables xi
    Aeq = [Aeq zeros(size(Aeq,1),1)]; % add a new column to Aeq
    A = [A zeros(size(A,1),1)]; %add a new column to A
    %add the constraints of the formula
    A = [A ; zeros(1,(k-1)*(nplaces+ntrans)) vi zeros(1,size(Aeq,2)-(k-1)*(nplaces+ntrans)-nplaces-1) -N]; %vi*m-N*xi \leq 0
    b = [b ; 0];
    A = [A ; zeros(1,(k-1)*(nplaces+ntrans)) -vi zeros(1,size(A,2)-(k-1)*(nplaces+ntrans)-nplaces-1) 1]; %xi-vi*m \leq 0
    b = [b ; 0];
end

%add constraints for infinite norm b
temp=[];
for i = 1 :k
    temp = [temp zeros(nplaces,nplaces) Post];
end
temp = [temp zeros(nplaces,2*length(props))];

A = [A zeros(size(A,1),1)]; %add a column of zeros corresponding to variable b
Aeq = [Aeq zeros(size(Aeq,1),1)]; %add a column of zeros corresponding to variable b

A = [A; temp -ones(nplaces,1)];
b = [b;zeros(nplaces,1)];

%%%%%%%%%%% cost function
data = get(gcf,'UserData');
cost = [];
for i = 1 : k
    cost = [cost zeros(1,nplaces) i*data.optim.param_boolean.lambda*ones(1,ntrans)];
end
cost = [cost zeros(1,2*length(props)) data.optim.param_boolean.mu];


%parse formula and convert to linear constraints/restrictions
[A,b] = rmt_formula2constraints(formula, A,b,length(props));

