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

function [A,b,negated_trajectory_alone,A_r,b_r] = rmt_formula2constraints(F, A,b,nr_props)
% parse Boolean formula (F) given in Conjunctive normal form
% convert formula to constraints for ILP (by adding rows in A,b,Aeq,beq on which ILP is run)

%nr_props - number of propositions/regions (not all have to be used in formula F)

% formula is a string of characters
% connectives: & - conjunction, | - disjunction, ! - negation
% literals: small or caps letters "y" and "Y" with number of prop. (e.g. y1, y2, Y1, ...) -small letters will mean requirement in final state, caps along trajectory
%example of formula: 'Y1&(y2|!y3)&(!Y3|!Y4)&(!y1)'   ( old version was with 'A&(b|!c)&(!C|!D)&(!a)' )

% fprintf('\nInput Boolean formula in Conjunctive Normal Form;\n\tUse & , | , ! for conjunction, disjunction and negation, respectively;\n');
% fprintf('\tUse letters for atomic propositions; upper case letter means restriction along trajectory, lower case restriction in final state.\n');
% 
% F = input('Formula: \n', 's');

F=strrep(F,' ',''); %remove spaces
F=strrep(F,'''',''); %remove apostrophes (if there are at beginning/end)

if ~isempty(setdiff(unique(F),'!&|()Yy0123456789'))
    uiwait(errordlg(sprintf('Boolean formula should contain y1, y2, ..., Y1, ... as atomic propositions, and characters !, &, |, (, )'), 'Robot Motion Toolbox','modal'));
    return;
end

% [D,F]=strtok(F,'&');    %D is current disjunction, F is remainder (F now begins with "&")

D=textscan(F,'%s','delimiter','&');  %separate disjunctions in cell D
D=D{1}; %D{i} will be a char string for i^th disj.

%while parsing, store restrictions for ILP (will be added in inequality Ax<=b); A_r will have nr_props columns and number of rows given by number of disjunctions
%first nr_props columns reffer to variables along trajectory, last nr_props columns are for final state
A_r=zeros(length(D),2*nr_props); %restrictions/constraints that will be added to Aeq
b_r=-ones(length(D),1); %corresponding for b (AX<=b); all b begin with value -1 and will be adjusted based on formula

negated_trajectory_alone = 1;
for i=1:length(D)   %current disjunction is string D{i}
    D{i}=strrep(D{i},'(',''); %remove paranthesis (may be only at first and last positions)
    D{i}=strrep(D{i},')','');
    prop=textscan(D{i},'%s','delimiter','|');  %elements/propositions of currrent disjunction (each is literal or negated literal, because F is in CNF)
    prop=prop{1};
    %find restrictions in current disjunction:
%     fprintf('\nDisjunction %d:\n',i);
    %in restrictions we will modify A_r(i,:) and b_r(i)
    for j=1:length(prop)    %current atomic prop is disj{j}
        %prop should be of kind "[!]yi" ([!] - ! appears or not)
        atom_pr=regexp(prop{j},'([!yY]+\d+)','tokens'); %separate in atomic propositions (possibly preceded by !)
        atom_pr=[atom_pr{:}];

        if length(atom_pr)~=1    %tests for something wrong in input form - there should be only one prop. here
            uiwait(errordlg(sprintf('Please input formula in CNF, with atomic props denoted by ''y1, y2, Y1, ...'', and negation can precede at most once a literal.\n"%s"',prop{j}),...
                'Robot Motion Toolbox','modal'));
            return;
        end
        atom_pr=atom_pr{1}; %string
        
        %find restrinction in part (literal) j of disjunction D{i}:
        if (atom_pr(1)~='!')   %same as isempty(strfind(atom_pr,'!')) %current atomic prop is not negated
            if isstrprop(atom_pr(1),'lower')    %lower case letter -> final state restriction
                prop_ind=str2double(atom_pr(2:end));  %index of proposition
%                 fprintf('\tAtomic prop. %d should be TRUE in final state\n',prop_ind);
                %modify constraint (last nr_props columns are for final state restrictions):
                A_r(i,nr_props+prop_ind)=-1;    %-1*x_i ... <=-1
                %no modification in b for non-negated prop.
                
            elseif isstrprop(atom_pr(1),'upper')    %upper case letter -> restriction along trajectory
                prop_ind=str2double(atom_pr(2:end));  %index of proposition
%                 fprintf('\tAtomic prop. %d should be TRUE along traj.\n',prop_ind);
                %modify constraint (first nr_props columns are for trajectory restrictions):
                A_r(i,prop_ind)=-1;    %-1*X_i ... <=-1
                %no modification in b for non-negated prop.
            else
                uiwait(errordlg(sprintf('Something wrong in disjunction "%s"',prop{j}),...
                    'Robot Motion Toolbox','modal'));
                return;
            end
            
        else  %negation
            if isstrprop(atom_pr(2),'lower')    %lower case letter -> final state restriction
                prop_ind=str2double(atom_pr(3:end));  %index of proposition
%                 fprintf('\tAtomic prop. %d should be FALSE in final state\n',prop_ind);
                %modify constraint (final state restrictions):
                A_r(i,nr_props+prop_ind)=1;    %1*x_i ... <=-1 +1
                b_r(i)=b_r(i)+1;    %add 1 to b for each negation
                
            elseif isstrprop(atom_pr(2),'upper')    %upper case letter -> restriction along trajectory
                prop_ind=str2double(atom_pr(3:end));  %index of proposition
%                 fprintf('\tAtomic prop. %d should be FALSE along traj.\n',prop_ind);
                %modify constraint (trajectory restrictions):
                A_r(i,prop_ind)=1;    %1*x_i ... <=-1 +1
                b_r(i)=b_r(i)+1;    %add 1 to b for each negation
                if (length(prop) > 1)
                    negated_trajectory_alone = 0;
                end
                
            else
                uiwait(errordlg(sprintf('Something wrong in disjunction "%s"',prop{j}),...
                    'Robot Motion Toolbox','modal'));
                return;
            end
        end
    end
end

%add restrictions to A,b:
A=[A; zeros(length(D),size(A,2)-2*nr_props-1) , A_r, zeros(length(D),1)];
b=[b; b_r];
