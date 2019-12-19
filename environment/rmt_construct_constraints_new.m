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

function [A,b,Aeq,beq,cost] = rmt_construct_constraints_new(Pre,Post,m0, props, formula)

%the variables are: [m_1 sigma_1 m_2 sigma_2 no_of_binary_variables_traj no_of_binary_variables_final b(reduce congestion)]

nplaces = size(Pre,1); %number of places
ntrans = size(Pre,2); % number of transitions

%add the state equation: m_1 = m_0 + (Post-Pre)*sigma_1
Aeq = [eye(nplaces) -(Post-Pre) zeros(nplaces,nplaces+ntrans)];
beq = m0;
%add the state equation: m_2 = m_1 + (Post-Pre)*sigma_2
Aeq = [Aeq ; -eye(nplaces) zeros(nplaces,ntrans) eye(nplaces) -(Post-Pre)];
beq = [beq ; zeros(nplaces,1)];
A=[];
b=[];
N = sum(m0); % number of robots


%define the boolean variables for the trajectories
for i = 1 : length(props)
    %define the vectors v
    vii = zeros(1,size(Aeq,2)); %intermediate state
    vii(props{i})=1;
    % add the boolean variables xi
    Aeq = [Aeq zeros(size(Aeq,1),1)]; % add a new column to Aeq
    if isempty(A)
        A = zeros(0,size(Aeq,2));
    else
        A = [A zeros(size(A,1),1)]; %add a new columns to A
    end
    %add the constraints of the formula for intermediate state
    A = [A ; vii -N]; %sum(vii*m_1)-N*xi \leq 0
    b = [b ; 0];
    A = [A ; -vii 1]; %xi-sum(vii*m_i) \leq 0
    b = [b ; 0];

    %add the constraints on the sigma_2 from intermediate marking to final
    %one
    vifs = zeros(1,nplaces);
    vifs(props{i})=1;
    A = [A ; zeros(1,nplaces+ntrans+nplaces) vifs*Pre zeros(1,size(A,2)-2*nplaces-2*ntrans-1) -N];
    b = [b; 0];
    A = [A; zeros(1,nplaces+ntrans+nplaces) -vifs*Pre zeros(1,size(A,2)-2*nplaces-2*ntrans-1) 1];
    b = [b; 0];

   %add the constraints on the sigma_1 from initial marking to intermediate
    %one
    vifs = zeros(1,nplaces);
    vifs(props{i})=1;
    A = [A ; zeros(1,nplaces) vifs*Post zeros(1,size(A,2)-2*nplaces-2*ntrans-1) zeros(1,ntrans+nplaces) -N];
    b = [b; -vifs*m0];
    A = [A; zeros(1,nplaces) -vifs*Post zeros(1,size(A,2)-2*nplaces-2*ntrans-1) zeros(1,ntrans+nplaces) 1];
    b = [b; vifs*m0];

end

%define the boolean variables for the final state
for i = 1 : length(props)
    %define the vectors v
    vif = zeros(1,size(Aeq,2)); %final state
    vif(props{i}+(nplaces+ntrans))=1;
    % add the boolean variables xi
    Aeq = [Aeq zeros(size(Aeq,1),1)]; % add a new column to Aeq
    if isempty(A)
        A = zeros(0,size(Aeq,2));
    else
        A = [A zeros(size(A,1),1)]; %add a new columns to A
    end
    %add the constraints of the formula for final state
    A = [A ; vif -N]; %sum(vii*m_1)-N*xi \leq 0
    b = [b ; 0];
    A = [A ; -vif 1]; %xi-sum(vii*m_i) \leq 0
    b = [b ; 0];
    
end




%add constraints for infinite norm b to reduce congestions
A = [A zeros(size(A,1),1)]; %add a column of zeros corresponding to variable b
Aeq = [Aeq zeros(size(Aeq,1),1)]; %add a column of zeros corresponding to variable b

A = [A; zeros(nplaces,nplaces) Pre zeros(nplaces,nplaces) Pre  zeros(nplaces,2*length(props)) -ones(nplaces,1)];
b = [b;zeros(nplaces,1)];

%%%%%%%%%%% cost function
cost = [zeros(1,nplaces) ones(1,ntrans) zeros(1,nplaces) ones(1,ntrans) zeros(1,2*length(props)) N+1];

%parse formula and convert to linear constraints/restrictions
[A,b] = rmt_formula2constraints(formula, A,b,length(props));

