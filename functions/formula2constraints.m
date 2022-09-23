function [A,b] = formula2constraints(F, nr_props)

% parse Boolean formula (F) given in Conjunctive normal form
% convert formula to constraints for ILP (by adding rows in A,b,Aeq,beq on which ILP is run)

%nr_props - number of propositions/regions (not all have to be used in formula F)

% formula is a string of characters
% connectives: & - conjunction, | - disjunction, ! - negation
% literals: caps letters (a-z, A-Z)
%example of formula: 'A&(!B|!C)'

% fprintf('\nInput Boolean formula in Conjunctive Normal Form;\n\tUse & , | , ! for conjunction, disjunction and negation, respectively;\n');
% fprintf('\tUse letters for atomic propositions; upper case letter means restriction along trajectory, lower case restriction in final state.\n');
% 
% F = input('Formula: \n', 's');
F = upper(F);
F=strrep(F,' ',''); %remove spaces
F=strrep(F,'''',''); %remove apostrophes (if there are at beginning/end)

D=textscan(F,'%s','delimiter','&');  %separate disjunctions in cell D
D=D{1}; %D{i} will be a char string for i^th disj.

%while parsing, store restrictions for ILP (will be added in inequality Ax<=b); A_r will have nr_props columns and number of rows given by number of disjunctions
%first nr_props columns reffer to variables along trajectory, last nr_props columns are for final state
A_r=zeros(length(D),nr_props); %restrictions/constraints that will be added to Aeq
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
        if length(disj{j})>2 || length(disj{j})==0    %tests for something wrong in input form
            fprintf('\n ERROR - please input formula in Closed Normal Form (conjunction of disjunctions, where negation can precede at most once a literal.\n');
            fprintf('%s',disj{j});
            return;
        elseif length(disj{j})==2 && ~(disj{j}(1)=='!' && isletter(disj{j}(2)))
            fprintf('\n ERROR - no letter or negation in disjunction "%s".\n',disj{j});
            return;
        elseif length(disj{j})==1 && ~isletter(disj{j}(1))
            fprintf('\n ERROR - no letter in disjunction "%s".\n',disj{j});
            return;
        end
        
        %find restrinction in part (literal) j of disjunction D{i}:
        if length(disj{j})==1   %non-negated letter (atomic proposition
           if isstrprop(disj{j}(1),'upper')    %upper case letter -> restriction along trajectory
                prop_ind=disj{j}(1)-'A'+1;  %index of proposition
%                 fprintf('\tAtomic prop. %d should be TRUE along traj.\n',prop_ind);
                %modify constraint (first nr_props columns are for trajectory restrictions):
                A_r(i,prop_ind)=-1;    %-1*X_i ... <=-1
                %no modification in b for non-negated prop.
                
            else
                fprintf('\nERROR - no letter in disjunction "%s"\n',disj{j});
                return;
            end
            
        else % length(disj{j})==2 && disj{j}(1)=='!'  %negation
            if isstrprop(disj{j}(2),'upper')    %upper case letter -> restriction along trajectory
                prop_ind=disj{j}(2)-'A'+1;  %index of proposition
%                 fprintf('\tAtomic prop. %d should be FALSE along traj.\n',prop_ind);
                %modify constraint (trajectory restrictions):
                A_r(i,prop_ind)=1;    %1*x_i ... <=-1 +1
                b_r(i)=b_r(i)+1;    %add 1 to b for each negation
                
            else
                fprintf('\nERROR - no letter in disjunction "%s"\n',disj{j});
                return;
            end
        end
    end
end
A=A_r;
b=b_r;
return