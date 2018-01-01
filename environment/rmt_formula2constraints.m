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

function [A,b] = rmt_formula2constraints(F, A,b,nr_props)
% parse Boolean formula (F) given in Conjunctive normal form
% convert formula to constraints for ILP (by adding rows in A,b,Aeq,beq on which ILP is run)

%nr_props - number of propositions/regions (not all have to be used in formula F)

% formula is a string of characters
% connectives: & - conjunction, | - disjunction, ! - negation
% literals: small or caps letters (a-z, A-Z) (small letters will mean requirement in final state, caps along trajectory)
%example of formula: 'A&(b|!c)&(!C|!D)&(!a)'

% fprintf('\nInput Boolean formula in Conjunctive Normal Form;\n\tUse & , | , ! for conjunction, disjunction and negation, respectively;\n');
% fprintf('\tUse letters for atomic propositions; upper case letter means restriction along trajectory, lower case restriction in final state.\n');
% 
% F = input('Formula: \n', 's');

F=strrep(F,' ',''); %remove spaces
F=strrep(F,'''',''); %remove apostrophes (if there are at beginning/end)

% [D,F]=strtok(F,'&');    %D is current disjunction, F is remainder (F now begins with "&")

D=textscan(F,'%s','delimiter','&');  %separate disjunctions in cell D
D=D{1}; %D{i} will be a char string for i^th disj.

%while parsing, store restrictions for ILP (will be added in inequality Ax<=b); A_r will have nr_props columns and number of rows given by number of disjunctions
%first nr_props columns reffer to variables along trajectory, last nr_props columns are for final state
A_r=zeros(length(D),2*nr_props); %restrictions/constraints that will be added to Aeq
b_r=-ones(length(D),1); %corresponding for b (AX<=b); all b begin with value -1 and will be adjusted based on formula

for i=1:length(D)   %current disjunction is string D{i}
    D{i}=strrep(D{i},'(',''); %remove paranthesis (may be only at first and last positions)
    D{i}=strrep(D{i},')','');
    disj=textscan(D{i},'%s','delimiter','|');  %elements of currrent disjunction (each is literal or negated literal, because F is in CNF)
    disj=disj{1};
    %find restrictions in current disjunction:
%     fprintf('\nDisjunction %d:\n',i);
    %in restrictions we will modify A_r(i,:) and b_r(i)
    for j=1:length(disj)    %current literal is disj{j}
        if length(disj{j})>2 || isempty(disj{j})    %tests for something wrong in input form
            uiwait(errordlg(sprintf('Please input formula in CNF (conjunction of disjunctions, where negation can precede at most once a literal.\n"%s"',disj{j}),...
                'Robot Motion Toolbox','modal'));
            return;
        elseif length(disj{j})==2 && ~(disj{j}(1)=='!' && isletter(disj{j}(2)))
            uiwait(errordlg(sprintf('No letter or negation in disjunction "%s"',disj{j}),...
                'Robot Motion Toolbox','modal'));
            return;
        elseif length(disj{j})==1 && ~isletter(disj{j}(1))
            uiwait(errordlg(sprintf('No letter in disjunction "%s"',disj{j}),...
                'Robot Motion Toolbox','modal'));
            return;
        end
        
        %find restrinction in part (literal) j of disjunction D{i}:
        if length(disj{j})==1   %non-negated letter (atomic proposition
            if isstrprop(disj{j}(1),'lower')    %lower case letter -> final state restriction
                prop_ind=disj{j}(1)-'a'+1;  %index of proposition
%                 fprintf('\tAtomic prop. %d should be TRUE in final state\n',prop_ind);
                %modify constraint (last nr_props columns are for final state restrictions):
                A_r(i,nr_props+prop_ind)=-1;    %-1*x_i ... <=-1
                %no modification in b for non-negated prop.
                
            elseif isstrprop(disj{j}(1),'upper')    %upper case letter -> restriction along trajectory
                prop_ind=disj{j}(1)-'A'+1;  %index of proposition
%                 fprintf('\tAtomic prop. %d should be TRUE along traj.\n',prop_ind);
                %modify constraint (first nr_props columns are for trajectory restrictions):
                A_r(i,prop_ind)=-1;    %-1*X_i ... <=-1
                %no modification in b for non-negated prop.
                
            else
                uiwait(errordlg(sprintf('No letter in disjunction "%s"',disj{j}),...
                    'Robot Motion Toolbox','modal'));
                return;
            end
            
        else % length(disj{j})==2 && disj{j}(1)=='!'  %negation
            if isstrprop(disj{j}(2),'lower')    %lower case letter -> final state restriction
                prop_ind=disj{j}(2)-'a'+1;  %index of proposition
%                 fprintf('\tAtomic prop. %d should be FALSE in final state\n',prop_ind);
                %modify constraint (final state restrictions):
                A_r(i,nr_props+prop_ind)=1;    %1*x_i ... <=-1 +1
                b_r(i)=b_r(i)+1;    %add 1 to b for each negation
                
            elseif isstrprop(disj{j}(2),'upper')    %upper case letter -> restriction along trajectory
                prop_ind=disj{j}(2)-'A'+1;  %index of proposition
%                 fprintf('\tAtomic prop. %d should be FALSE along traj.\n',prop_ind);
                %modify constraint (trajectory restrictions):
                A_r(i,prop_ind)=1;    %1*x_i ... <=-1 +1
                b_r(i)=b_r(i)+1;    %add 1 to b for each negation
                
            else
                uiwait(errordlg(sprintf('No letter in disjunction "%s"',disj{j}),...
                    'Robot Motion Toolbox','modal'));
                return;
            end
        end
    end
end

%add restrictions to A,b:
A=[A; zeros(length(D),size(A,2)-2*nr_props-1) , A_r, zeros(length(D),1)];
b=[b; b_r];
