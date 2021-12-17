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
%   Last modification Enero, 2020.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function [A,b,Aeq,beq,cost] = rmt_construct_constraints_ltl(Pre,Post,m0, nplaces_orig, ntrans_orig, nplaces_observ, k, final)
trans_model = [zeros(1,length(m0)) ones(1,ntrans_orig) zeros(1,size(Pre,2)-ntrans_orig)];
trans_Buchi = [zeros(1,length(m0)) zeros(1,ntrans_orig) ones(1,size(Pre,2)-ntrans_orig)];
marking_final = sparse(zeros(1,length(m0)));
marking_final(final)=1;

%the variables are: [m_i sigma_i ], i = 1,...,k

nplaces = size(Pre,1); %number of places
ntrans = size(Pre,2); % number of transitions

% a), b)
%add the state equation: m_{i+1} = m_i + (Post-Pre)*sigma_{i+1}
Aeq = [eye(nplaces) -(Post-Pre)];
beq = m0;
A = [zeros(nplaces,nplaces) Pre]; %m0 - Pre \cdot sigma \geq 0
b = m0;
%in the first step fire only transitions of the robot model
Aeq = [Aeq; trans_Buchi];
beq = [beq;0];

for i = 2 : k
    Aeq = [Aeq zeros(size(Aeq,1),nplaces+ntrans)]; %add nplaces+ntrans columns to Aeq
    Aeq = [Aeq; zeros(nplaces,(i-2)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) eye(nplaces) -(Post-Pre)]; %add the state equation
    beq = [beq;zeros(nplaces,1)];
    % c), d)
    if (i/2 == round(i/2)) %fire only transitions of the Buchi automaton
        Aeq = [Aeq; zeros(1,(i-1)*(nplaces+ntrans)) trans_model]; %not fire transition of the model
        beq = [beq;0];
        Aeq = [Aeq; zeros(1,(i-1)*(nplaces+ntrans)) trans_Buchi]; %fire one transition of Buchi
        beq = [beq;1];
    % f)     
    else %fire only transitions of the robot model
        Aeq = [Aeq; zeros(1,(i-1)*(nplaces+ntrans)) trans_Buchi];
        beq = [beq;0];
    end
    A = [A zeros(size(A,1),nplaces+ntrans)]; %add nplaces+ntrans columns to A
    A = [A ; zeros(nplaces, (i-2)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) zeros(nplaces,nplaces) Pre];
    b = [b ; zeros(nplaces,1)];
end

% this should not be used 
%put that an intermediate marking of Buchi is equal with a final one in the final
interm = round(k/2);
% m(interm) == m_final

nplaces_buchi = nplaces-nplaces_orig-nplaces_observ;

Aeq=[Aeq; zeros(nplaces_buchi,(interm-1)*(nplaces+ntrans)) zeros(nplaces_buchi,nplaces_orig+nplaces_observ) eye(nplaces_buchi) ...
    zeros(nplaces_buchi,ntrans) zeros(nplaces_buchi,(k-interm-1)*(nplaces+ntrans)) zeros(nplaces_buchi,nplaces_orig+nplaces_observ) ...
    -eye(nplaces_buchi) zeros(nplaces_buchi,ntrans)];
beq = [beq ; zeros(nplaces_buchi,1)];

% g) new f*mB = 1
% -m_final * 1 <=-1
% final marking in Buchi needs to be equal with 1
Aeq = [Aeq; zeros(1,(k-1)*(nplaces+ntrans)) -marking_final zeros(1,ntrans)];
beq = [beq;-1];

%move at least one robot in the first step
% A = [A; zeros(1,nplaces) -trans_model zeros(1,(k-1)*(nplaces+ntrans))];
% b = [b ; -1];

%fire at lesat one transiton of the Buchi in last step 
%A = [A; zeros(1,(k-1)*(nplaces+ntrans)) -trans_Buchi];
%b = [b ; -1];

%fprintf(1,'\nIntermediate states %d',interm);
%%%%%%%%%%% cost function
cost = [];
for i = 1 : k
    cost = [cost zeros(1,nplaces) ones(1,ntrans)];
end



