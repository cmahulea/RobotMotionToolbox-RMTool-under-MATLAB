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
%Convert LTL formula to a Buchi automaton -> version April 2018

%inputs:
%alphabet of Buchi is Obs (matrix) - observable set of transition system used, a row of Obs is an observable (can be a tuple of some elements)
%formula: string containing LTL formula (all its atomic proposition are in alphabet from which alphabet_set was created) (use spaces in formula)
%atomic propositions are of form y0...0x, where x is an integer; true, false
% USE "y" for denoting propositions (outputs of system)
%Boolean operators: ! - negation; & - and; | - or; -> - implication; <-> - equivalence;
%Temporal operators: U - until; R - release; F - eventually; G - always; X - next
%example of formula: (F y1) & G !(y2 | y3)

%output: cell aray containing Buchi automaton accepting exactly the infinite strings satisfying LTL formula

%this function will transform the automaton in text form returned by LTL2BA (parsing) to a structure (tuple) (B=(S,S0,trans,F), where:
%S-states (identified by integers 1,2,...; S0-initial state(s); F-accepting (final) state(s);
%trans-structure containing transitions (trans{i,k} is a vector showing which element(s) (row indices in Obs) of alphabet_set enable transition s_i->s_k)

%make changes in formula string so it matches the sintax required by ltl2ba.exe:
formula = regexprep(formula,'[pPoOY]','y'); %if user mistakenly uses for props letters p, P, o, O, or Y -> replace with lower-case "y"
formula = regexprep(formula,'[^y0-9()<>\-!&\|FGU]',''); %remove any character different than "y", number, round paranthesis, symbols for Boolean/temporal
formula=regexprep(formula, '&&', '&');  %if user used double "&&" instead of a single
formula=regexprep(formula, '&', ' && '); %for ltl2ba
formula=regexprep(formula, '\|\|', '|'); %if user used double "||" instead of a single (| has special meaning, so it's preceded by \)
formula=regexprep(formula, '\|', ' || ');
formula=regexprep(formula, 'U', ' U ');
formula=regexprep(formula, 'R', ' V ');
formula=regexprep(formula, 'F', ' <> ');
formula=regexprep(formula, 'G', ' [] ');
formula=regexprep(formula, '  ', ' ');

temp = computer;
try
    if ~isempty(findobj('type','figure'))   %if there exists at least one figure (otherwise, function may have been called outside RMTool)
        data = get(gcf,'UserData');
    end
    runLTL2BA = data.ltl2ba;    %forces "catch" if no figure
catch
    if (strcmpi(temp,'PCWIN') || strcmpi(temp,'PCWIN64'))
        [~, WindowsVersion] = system('ver');
        if isempty(strfind(WindowsVersion,'10')) %different than Windows 10 (7 or before)
            runLTL2BA = ['.' filesep 'aux_toolboxes' filesep 'ltl2ba' filesep 'ltl2ba_Win7.exe'];
        else %Windows 10
            runLTL2BA = ['.' filesep 'aux_toolboxes' filesep 'ltl2ba' filesep 'ltl2ba.exe'];
        end
    elseif strcmpi(temp,'GLNXA64')
        runLTL2BA = ['.' filesep 'aux_toolboxes' filesep 'ltl2ba' filesep 'ltl2bal'];
    elseif strcmpi(temp,'MACI64')
        runLTL2BA = ['.' filesep 'aux_toolboxes' filesep 'ltl2ba' filesep 'ltl2ba'];
    end
    fprintf(1,'\n Note: rmt_create_buchi is trying the default path for ltl2ba (subfolder aux_toolboxex -> ltl2ba)...\n');
end

if (strcmpi(temp,'PCWIN') || strcmpi(temp,'PCWIN64'))
    [s,r]=dos([runLTL2BA ' -d -f "' formula '"']); %sintax for calling ltl2ba.exe (located in subdir .\ltl2ba); use full description result (-d)
    %example of run: [s,r]=dos('ltl2ba -d -f "<> p1"'); %LTL 2 Buchi for our formula
elseif strcmpi(temp,'GLNXA64')
    [s,r]=unix([runLTL2BA ' -d -f "' formula '"']);
elseif strcmpi(temp,'MACI64')
    [s,r]=unix([runLTL2BA ' -d -f "' formula '"']);
end

if s~=0 %error
    message = sprintf('ERROR when converting LTL to Buchi. Posible causes:\n (1) Antivirus may have stopped ltl2ba -> temporarily disable antivirus.\n (2) Path to LTL2BA not correct -> make sure current folder is the RMTool one (where file rmt.m is located) and the original subfolders were not renamed.\n (3) If calling rmt_create_buchi outside RMTool, please change path to ltl2ba folder on lines 64-71 in file rmt_create_buchi.m.\n');
    uiwait(errordlg(message,'Robot Motion Toolbox','modal'));
    error(message);
    B=[];
    return
end

%s=0, conversion successfull

%isolate text part which we will parse for creating B
str1=[char(10), 'never {'];    %first string for match (start of interesting "zone" from what ltl2ba returns); char(10) - newline -> ASCII table https://www.techonthenet.com/ascii/chart.php
%str1=[char(10), char(10), 'never {'];
str2=[char(10), '}'];    %second string for match (end of interesting "zone" from what ltl2ba returns)
%remove from beginning until line contained by str1 (including it), and from beginning of str2 to end 
ind1=strfind(r,str1);
ind2=strfind(r,str2);
ind2(ind2 <= ind1(end))=[];
r_temp=r(ind1(end):end);  %used below, if there are no accepting states
r([1:ind1(end) , (ind2(1)+1):end])=[]; %leave last newline, will be usefull below in finding "row"

% we have a string like: "never { /* <>(p1||p3) && <>(p2&&!p1) */
%                             T0_init:
%                             	  if
%                             	  :: (1) -> goto T0_init
%                                 :: (!p1 && p2) -> goto T0_S2
%                                 fi;
%                             T0_S2:
%                                 if
%                                 . . .
%                             accept_all:
%                                 skip
%                            "
% we want to create a cell B containing the Buchi automaton (see help on Characters and Strings, Regular Expressions)

if ~isempty(regexp(r_temp,'empty automaton', 'once' ))
    fprintf('\nERROR - empty Buchi automaton (maybe LTL formula has logical inconsistencies)\n');
    B=[];
    return
end
if isempty(regexp(r,'accept', 'once' ))
    fprintf('\nERROR - no accepting state in Buchi automaton (maybe LTL formula has logical inconsistencies)\n');
    B=[];
    return
end

ind1=strfind(r,char(10)); %remove first line (with "never...")
r(1:ind1(1))=[];
%remove lines with "if", with "fi;", some tabs and unusefull sequences:
ind=strfind(r,[char(9), 'if', char(10)]); %after some states (char(9) is tab)
for i=length(ind):-1:1  %go backwards, because otherwise would affect following instances
    r(ind(i):ind(i)+3)=[];
end
ind=strfind(r,[char(9), 'fi;', char(10)]); %after some states (char(9) is tab)
for i=length(ind):-1:1
    r(ind(i):ind(i)+4)=[];
end
ind=strfind(r,[char(9), ':: ']); %remove (tabs followed by :: ) that appear after some states (char(9) is tab)
for i=length(ind):-1:1
    r(ind(i):ind(i)+3)=[];
end
r(strfind(r,char(9)))=[]; %remove all remaining tabs
ind=strfind(r,'goto '); %remove "goto " (is indicated and preceeded anyway by "->"
for i=length(ind):-1:1
    r(ind(i):ind(i)+4)=[];
end


%Buchi states and their names allocated by ltl2ba
[S_names, s_ind, e_ind]=regexp(r,'(\w*):\n','tokens'); %state names (any string followed by ":" and immediately after "\n"
S_names=[S_names{:}]; %get rid of sub-cells (S_names is a cell array of strings)

states_no=length(S_names);  %number of states in Buchi aut
B.S=1:states_no;    %numeric indices for states

%initial and final(accepting) states:
B.S0=find( cellfun( 'isempty', regexp(S_names,'init') ) == 0 );    %find indices of state(s) containing word 'init' (initial st.)
B.F=find( cellfun( 'isempty', regexp(S_names,'accept') ) == 0 );    %find indices of state(s) containing word 'accept' (accepting states)

%find transitions (search succesors for each state and atomic proposition) 
B.trans=cell(states_no,states_no);  %trans(i,next_state) gives the indices of elements from Obs that enable transition s_i -> s_{next_state}
for i=1:states_no
    if i~=states_no
        str=r((e_ind(i)+1): (s_ind(i+1)-1));   %select string containing transitions of current state (y1 -> . . .); r(e_ind(i)) is newline
    else    %last state in string r
        str=r((e_ind(i)+1): end);
    end
    
    % rows=regexp(r,'.*','match','dotexceptnewline'); %cell array with rows of text
    % %dot (.) matches every character, including the newline. Exclude newline characters from the match using the 'dotexceptnewline' option.
    row=regexp(str,'([^\n]*)\n','tokens');  %token: ([^\n] - any character different than new line), (* - any number of such characters)
    row=[row{:}]; %cell with rows for current state (i) (each row contains one state)
    for j=1:length(row)
        if strcmp(row{j},'skip')    %self-loop for all inputs
            B.trans{i,i}=1:size(Obs,1);   %for all observables there is loop s_i -> s_i
            B.trans{i,i}=B.trans{i,i}(:);   %force column vector (or "B.trans{i,i}=reshape(B.trans{i,i},[],1)")
            continue; %end current tests for this row, continue with next row or next state
        end
        
        % next_state=strmatch(row{j}((strfind(row{j},' -> ')+4) : end), S_names, 'exact'); %find string with state name and find index for exact matching in possible names
        next_state=find(strcmp(row{j}((strfind(row{j},' -> ')+4) : end), S_names)); %find string with state name and find index for exact matching in possible names
                
        %row{j} is in Disjunctive Normal Form (DNF); isolate terms in disjunction
        dnf_expr=row{j}(1 : (strfind(row{j},' -> ')-1));   %DNF expression
        conj_props=regexp([dnf_expr,' ||'],'([^|]*) ||','tokens');  %conjunction props (at least one, since apending ' ||' at end of dnf_expr
        conj_props=[conj_props{:}];
        
        for k=1:length(conj_props)
            prop=conj_props{k}; %current proposition (term from DNF)
            prop(strfind(prop,' '))=[]; %remove all spaces
            prop(strfind(prop,'('))=[]; %remove all paranthesis
            prop(strfind(prop,')'))=[]; %remove all paranthesis
            
            %ONLY {& (and), ! (not), 1 (any=True)} can appear on each row with respect to propositions (|| (OR) operator results in 2 rows)
            %if 1 appears, it is the first element and there is no atomic proposition on current row
            if prop(1)==1
                B.trans{i,next_state}=1:size(Obs,1);   %for all observables there is transition s_i -> s_{next_state}
                B.trans{i,next_state}=B.trans{i,next_state}(:);   %force column vector
                continue
            end

            %current row does not start with 1; prop is only of kind "[!]yi&&[!]yj&&[!]yk" ([!] - ! appears or not)

            atom_pr=regexp(prop,'([!y]+\d+)','tokens'); %separate in atomic propositions (possibly preceded by !)
            atom_pr=[atom_pr{:}];
            labels=1:size(Obs,1);  %will store indices of observables (alphabet elements) that enable the current transition
                         %start with all labels, and at each step keep (intersect with) those "accepted" by the current atomic prop
                         %(if atomic prop is not negated, observables should contain it and vice-cersa)
        
            for ap=1:length(atom_pr) %for each atomic prop, modify vector "labels"
                if isempty(strfind(atom_pr{ap},'!'))   %same as (atom_pr{ap}(1)~='!') %current atomic prop is not negated, so we keep ALL subsets that contain it
                                %use intersections because atomic propositions (possibly negated) are linked only by & (AND) operator
                    ap_num=str2double(atom_pr{ap}(2:end));  %numeric value of current atomic proposition (in range 1,...,N_s) (elimintate 'y' from beginning)
                    labels_temp=find(sum(ap_num==Obs,2));    %indices of rows from Obs (observables) containing current atomic proposition
                    labels=intersect(labels, labels_temp);    %update set of feasible indices (until now)

                else    %same as (atom_pr{ap}(1)=='!') %negated, find all subsets NOT containing the current atomic proposition
                    ap_num=str2double(atom_pr{ap}(3:end));  %numeric value of current atomic proposition (in range 1,...,N_s) (elimintate '!y' from beginning)
                    labels_temp=find(sum(ap_num==Obs,2)==0);    %indices of rows from Obs (observables) NOT containing current atomic proposition
                    labels=intersect(labels, labels_temp);    %update set of feasible indices (until now)
                end
            end

            B.trans{i,next_state}=union(B.trans{i,next_state},labels);    %add current labels to current transitions (transition s_i -> s_{next_state} can be captured by more rows
                                                         %(equivalent with OR operator between propositions)
            B.trans{i,next_state}=B.trans{i,next_state}(:);   %force column vector
        end
    end
end
