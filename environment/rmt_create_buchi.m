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

%ltl2ba.exe (Oddoux&Gastin, http://www.liafa.jussieu.fr/ltl2ba/, http://www.liafa.jussieu.fr/~oddoux/ltl2ba/)(compiled using Cygwin) is used to convert an LTL formula to a Buchi automaton;
%automaton is in text form;

%% ============================================================================
%   MOBILE ROBOT TOOLBOX
%   Graphical User Interface
%   First version released on September, 2014. 
%   Last modification December 29, 2015.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function B = rmt_create_buchi(formula, Obs)
%Convert LTL formula to a Buchi automaton

%inputs:
%alphabet of Buchi is Obs (matrix) - observable set of transition system used, a row of Obs is an observable (can be a tuple of some elements)
%formula: string containing LTL formula (all its atomic proposition are in alphabet from which alphabet_set was created) (use spaces in formula)
%atomic propositions are of form p0...0x, where x is an integer; true, false
%Boolean operators: ! - negation; & - and; | - or; -> - implication; <-> - equivalence;
%Temporal operators: U - until; R - release; F - eventually; G - always; X - next
%example of formula: (F p1) & G !(p2 | p3)

%output: cell aray containing Buchi automaton accepting exactly the infinite strings satisfying LTL formula

%this function will transform the automaton in text form returned by LTL2BA (parsing) to a structure (tuple) (B=(S,S0,trans,F), where:
%S-states (identified by integers 1,2,...; S0-initial state(s); F-accepting (final) state(s);
%trans-structure containing transitions (trans{i,k} is a vector showing which element(s) (row indices in Obs) of alphabet_set enable transition s_i->s_k)

formula=regexprep(formula, '&', '&&');  %make changes in formula string so it matches the sintax required by ltl2ba.exe
formula=regexprep(formula, '\|', '||'); %| has special meaning, so it's preceded by \
formula=regexprep(formula, 'R', 'V');
formula=regexprep(formula, 'F', '<>');
formula=regexprep(formula, 'G', '[]');

data = get(gcf,'UserData');
runLTL2BA = data.ltl2ba;
temp = computer;

if (strcmpi(temp,'PCWIN') || strcmpi(temp,'PCWIN64'))
    [s,r]=dos([runLTL2BA ' -d -f "' formula '"']); %sintax for calling ltl2ba.exe (located in subdir .\ltl2ba); use full description result (-d)
    %example of run: [s,r]=dos('ltl2ba -d -f "<> p1"'); %LTL 2 Buchi for our formula
elseif strcmpi(temp,'GLNXA64')
    [s,r]=unix([runLTL2BA ' -d -f "' formula '"']);
elseif strcmpi(temp,'MACI64')
    [s,r]=unix([runLTL2BA ' -d -f "' formula '"']);
end

if s~=0 %error
    message = sprintf('LTL to Buchi not corrected installed.\nRemove RMTconfig.txt file and run again the toolbox\nIt will be automatically created!');
    uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
    return
end

%s=0, conversion successfull
% sig=1:size(Obs,1);  %sigmas: numeric labels for observables (integers) (# of rows of matrix Obs)

str1=[char(10), 'Buchi automaton after simplification' char(10)];    %first string for match (char(10) - line feed)
%str2=[char(10), char(10), 'never {'];    %second string for match (end of interesting "zone" from what ltl2ba returns)
str2=[char(10), 'never {'];    %second string for match (end of interesting "zone" from what ltl2ba returns)

%remove from beginning until line contained by str1 (including it), and from beginning of str2 to end 
r_temp=r(findstr(r,str2):end);  %used below, if there are no accepting states
r([1:(findstr(r,str1)+length(str1)-1) , findstr(r,str2):end])=[];


% we have a string like: "state x
%                         p1 -> state_label
%                         . . .
%                         state accept_**
%                         . . .
%                         "
% we want to create a cell B containing the Buchi automaton (see help on Characters and Strings, Regular Expressions)

% states_no=length(findstr(r,'state'));  %number of states in Buchi aut
% B.S=1:states_no;
if ~isempty(regexp(r,'empty automaton', 'once' ))
    fprintf('\nERROR - empty Buchi automaton (maybe LTL formula has logical inconsistencies)\n');
    return
end
if isempty(regexp(r_temp,'accept', 'once' ))
    fprintf('\nERROR - no accepting state in Buchi automaton (maybe LTL formula has logical inconsistencies)\n');
    return
end

[S_names s_ind e_ind]=regexp(r, 'state (\S*)', 'tokens', 'start', 'end'); %S_names is a cell containing sub-cells with state names
                %s_ind, e_ind contain start&end indices for expressions (useful for delimiting parts of r corresponding to a certain state)
S_names=[S_names{:}];   %get rid of sub-cells (S_names is a cell array of strings)
states_no=length(S_names);  %number of states in Buchi aut
B.S=1:states_no;    %numeric indices for states conteining in S_names
B.S0=find( cellfun( 'isempty', regexp(S_names,'init') ) == 0 );    %find indices of state(s) containing word 'init' (initial st.)

%final(accepting) states:
if states_no==1
    B.F=1;  %if there is only one state it may not contain string "accept", however it is accepting if the above tests were passed
elseif states_no < 1000 %for number of states less than about 1000, find accepting states as follows (to not skip some)
    S_names_temp=regexp(r_temp,'(\w*):\n','tokens');
    S_names_temp=[S_names_temp{:}]; %same order as in S_names
    B.F=[];
    %find accepting states in r_temp (otherwise might miss one, if it's also initial)
    ind_fin=regexp(r_temp,[char(10) 'accept']);  %beginning positions of accept_statename
    for i=1:length(ind_fin)
        str_fin=r_temp(ind_fin(i):end); %look from current point on
        str_fin=str_fin(2 : (regexp(str_fin,':','once')-1));    %first char was new line, don't need it
        fin=strmatch(str_fin, S_names_temp, 'exact');
        B.F=[B.F fin]; %mette l'indice dove c'è scritto accept in B.F
    end
else    %for more than ~1000 states, some states (e.g. initial) wrongly include 'accept' in string 'r_temp', but they doesn't in S_names
    B.F=find( cellfun( 'isempty', regexp(S_names,'accept') ) == 0 );    %find indices of state(s) containing word 'accept' (accepting states)
end
%find transitions (search succesors for each state and atomic proposition)
B.trans=cell(states_no,states_no);  %trans(i,k) gives the indices of elements from alphabet_set (with "or" between them) for which s_i -> s_k
for i=1:states_no
    if i~=states_no
        str=r((e_ind(i)+2): (s_ind(i+1)-1));   %select string containing transitions of current state (p1 -> . . .)
    else    %last state in string r
        str=r((e_ind(i)+2): end);
    end

    row=regexp(str,'([^\n]*)\n','tokens');  %token: ([^\n] - any character different than new line), (* - any number of such characters)
    row=[row{:}]; %cell with rows for current state (i) (each row contains one state)
    for j=1:length(row)
        % k=find( cellfun( 'isempty', regexp(row{j}, S_names) ) == 0 ); %index of state in which s_i transit (on current row)
        %k should be an integer (could be vector, if there are states like T0_S1, T0_S11, but this happens for unusually large formulas
        %in order to avoid this, use the following line (instead the above commented one):
        k=strmatch(row{j}((findstr(row{j},' -> ')+4) : end), S_names, 'exact'); %find string with state name and find index for exact matching in possible names

       %ONLY {& (and), ! (not), 1 (any=True)} can appear on each row with respect to propositions (|| (OR) operator results in 2 rows)
       %if 1 appears, it is the first element and there is no atomic proposition on current row
        if row{j}(1)==1
            B.trans{i,k}=1:size(Obs,1);   %for all observables there is transition s_i -> s_k
            continue
        end

       %1 does not appear on current row
        prop=row{j}(1 : (findstr(row{j},' -> ')-1)); %delimitate proposition (expression) involving atomic propositions
                                                     %prop is only of kind "[!]pi & [!]pj & [!]pk" ([!] - ! appears or not)

        atom_pr=regexp(prop,'([!u]+\d+)','tokens'); %separate in atomic propositions (possibly preceded by !)
        atom_pr=[atom_pr{:}];
        labels=1:size(Obs,1);  %will store indices of observables (alphabet elements) that enable the current transition
                     %start with all labels, and at each step keep (intersect with) those "accepted" by the current atomic prop
                     %(if atomic prop is not negated, observables should contain it and vice-cersa)
        
        for ap=1:length(atom_pr) %for each atomic prop, modify vector "labels"
            if isempty(findstr(atom_pr{ap},'!'))   %same as (atom_pr{ap}(1)~='!') %current atomic prop is not negated, so we keep ALL subsets that contain it
                            %use intersections because atomic propositions (possibly negated) are linked only by & (AND) operator
                ap_num=str2double(atom_pr{ap}(2:end));  %numeric value of current atomic proposition (in range 1,...,N_s) (elimintate 'p' from beginning)
                labels_temp=find(sum(ap_num==Obs,2));    %indices of rows from Obs (observables) containing current atomic proposition
                labels=intersect(labels, labels_temp);    %update set of feasible indices (until now)

            else    %same as (atom_pr{ap}(1)=='!') %negated, find all subsets NOT containing the current atomic proposition
                ap_num=str2double(atom_pr{ap}(3:end));  %numeric value of current atomic proposition (in range 1,...,N_s) (elimintate '!p' from beginning)
                labels_temp=find(sum(ap_num==Obs,2)==0);    %indices of rows from Obs (observables) NOT containing current atomic proposition
                labels=intersect(labels, labels_temp);    %update set of feasible indices (until now)                
            end
        end

        B.trans{i,k}=union(B.trans{i,k},labels);    %add current labels to current transitions (transition s_i -> s_k can be captured by more rows
                                                    %(equivalent with OR operator between propositions)
    end
end

if isempty(B.F) %no final state detected (because it is only one initial state and ltl2ba names it only "init" in description of simplified autmaton)
    B.F=B.S0;
end
