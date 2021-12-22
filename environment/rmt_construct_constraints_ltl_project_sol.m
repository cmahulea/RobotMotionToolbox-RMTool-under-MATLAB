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

function [A,b,Aeq,beq,cost] = rmt_construct_constraints_ltl_project_sol(Pre,Post,m0, number_of_robots, possible_regions, nplaces, ntrans, data)

N_r = length(data.RO);

Aeq=[];
beq = [];
A=[];
b = [];
%variables: m1 sigma1 m2 sigma2 ....
Aeq = [eye(nplaces) -(Post-Pre)]; %state equation
beq = m0;

%in the first step, at m1 the same numbers of robots remains in he
%regions with the same observation as at m0
for i = 1 : length(possible_regions{1})
    temp = zeros(1,nplaces);
    temp(possible_regions{1}{i})=1;
    Aeq = [Aeq; temp zeros(1,ntrans)];
    beq = [beq; temp*m0];
end
other_regions = setdiff(data.T.Q,unique([possible_regions{1}{:}]));
not_fire = zeros(1,ntrans);
for k = 1 : length(other_regions)
    not_fire(find(Post(other_regions(k),:))) = 1;
end
Aeq = [Aeq; zeros(1,nplaces) not_fire];
beq = [beq; 0];

A = [zeros(nplaces,nplaces) Post]; %constraint for collision avoidance
b = [ones(nplaces,1)-m0];

%N_r - number of robots in order to obtain collision free trajectories
for j = 2 : N_r
    Aeq = [Aeq zeros(size(Aeq,1),nplaces+ntrans)];
    A = [A zeros(size(A,1),nplaces+ntrans)]; %add nplaces+ntrans columns corresponding to the new intermediate marking
    Aeq = [Aeq; zeros(nplaces,(j-2)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) eye(nplaces) -(Post-Pre)]; %state equation
    beq = [beq; zeros(nplaces,1)];
    %in the first step, at m1 the same numbers of robots remains in the
    %regions with the same observation as at m0
    for i = 1 : length(possible_regions{1})
        temp = zeros(1,nplaces);
        temp(possible_regions{1}{i})=1;
        %keep the same number of robots in the regions with observations
        Aeq = [Aeq; zeros(1,(j-2)*(nplaces+ntrans)) temp zeros(1,ntrans) -temp zeros(1,ntrans)];
        beq = [beq; 0];
    end
    %not fire transitions to change the regions and get other
    %observations
    Aeq = [Aeq; zeros(1,(j-1)*(nplaces+ntrans)) zeros(1,nplaces) not_fire];
    beq = [beq; 0];
    
    %constraint for collision avoidance
    A = [A; zeros(nplaces,(j-2)*(nplaces+ntrans)) eye(nplaces) zeros(nplaces,ntrans) zeros(nplaces,nplaces) Post];
    b = [b; ones(nplaces,1)];
end

% add a new marking in which each robot moves maximum one regions,
% at this marking active the observations
Aeq = [Aeq zeros(size(Aeq,1),nplaces+ntrans)];
A = [A zeros(size(A,1),nplaces+ntrans)]; %add nplaces+ntrans columns corresponding to the new intermediate marking
% fire only one transition
A = [A; zeros(nplaces,(N_r-1)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) zeros(nplaces,nplaces) +Pre];
b = [b;zeros(nplaces,1)];
%state equation
Aeq = [Aeq; zeros(nplaces,(N_r-1)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) eye(nplaces) -(Post-Pre)];
beq = [beq; zeros(nplaces,1)];
%change the regions to the next sets
for j = 1 : length(possible_regions{2})
    temp = zeros(1,nplaces);
    temp(possible_regions{2}{j})=1;
    Aeq = [Aeq; zeros(1,(N_r)*(nplaces+ntrans)) temp zeros(1,ntrans)];
    beq = [beq; number_of_robots{2}(j)];
end

for i = 2 : length(possible_regions)-1
    other_regions = setdiff(data.T.Q,unique([possible_regions{i}{:}]));
    not_fire = zeros(1,ntrans);
    for k = 1 : length(other_regions)
        not_fire(find(Post(other_regions(k),:))) = 1;
    end
    
    for j = 1 : N_r %N_r - number of robots in order to obtain collision free trajectories
        actual_int_mark = size(Aeq,2)/(nplaces+ntrans);
        %add nplaces+ntrans columns corresponding to the new intermediate marking
        Aeq = [Aeq zeros(size(Aeq,1),nplaces+ntrans)];
        A = [A zeros(size(A,1),nplaces+ntrans)];
        %state equation
        Aeq = [Aeq; zeros(nplaces,(actual_int_mark-1)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) eye(nplaces) -(Post-Pre)];
        beq = [beq; zeros(nplaces,1)];
        %keep the same observations
        for k = 1 : length(possible_regions{i})
            temp = zeros(1,nplaces);
            temp(possible_regions{i}{k})=1;
            Aeq = [Aeq; zeros(1,actual_int_mark*(nplaces+ntrans)) temp zeros(1,ntrans)];
            beq = [beq; number_of_robots{i}(k)];
        end
        %do not fire transitions that brings to places with other
        %observations
        Aeq = [Aeq; zeros(1,actual_int_mark*(nplaces+ntrans)) zeros(1,nplaces) not_fire];
        beq = [beq;0];
        %constraint for collision avoidance
        A = [A; zeros(nplaces,(actual_int_mark-1)*(nplaces+ntrans)) eye(nplaces) zeros(nplaces,ntrans) zeros(nplaces,nplaces) Post];
        b = [b; ones(nplaces,1)];
    end
    %add a new marking to perform a transition for Buchi
    %add nplaces+ntrans columns corresponding to the new intermediate marking
    Aeq = [Aeq zeros(size(Aeq,1),nplaces+ntrans)];
    A = [A zeros(size(A,1),nplaces+ntrans)];
    %advance only one transition
    A = [A; zeros(nplaces,(actual_int_mark)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) zeros(nplaces,nplaces) +Pre];
    b = [b;zeros(nplaces,1)];
    %state equation
    Aeq = [Aeq; zeros(nplaces,(actual_int_mark)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) eye(nplaces) -(Post-Pre)];
    beq = [beq; zeros(nplaces,1)];
    %constraints to activate the observation at the final state
    actual_int_mark = size(Aeq,2)/(nplaces+ntrans)-1;
    for k = 1 : length(possible_regions{i+1})
        temp = zeros(1,nplaces);
        temp(possible_regions{i+1}{k})=1;
        Aeq = [Aeq; zeros(1,actual_int_mark*(nplaces+ntrans)) temp zeros(1,ntrans)];
        beq = [beq; number_of_robots{i+1}(k)];
    end
end
cost = [];
for i = 1 : size(Aeq,2)/(nplaces+ntrans)
    cost = [cost zeros(1,nplaces) ones(1,ntrans)];
end

end
