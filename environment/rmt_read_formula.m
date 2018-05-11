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

function formula = rmt_read_formula(N_s)
%read an LTL formula and replace names of propositions (px -> p0..0x)
%LTL formula is over the set of feasible subpolytopes in partition
%N_s=number of subpolytopes from partition

%create alphabet:
pad_no=num2str(length(num2str(N_s)));   %number of positions to represent propositions no (e.g. for 12 props - 2 positions: p01,p02,...)
new_alph=cell(1,N_s);
alphabet=cell(1,N_s);

for i=1:N_s
    new_alph{i}=sprintf(['p%d'],i);    %struct with names of props (useful below for testing and replacing in formula)
    alphabet{i}=sprintf(['p%0' pad_no 'd'],i);   %alphabet; we add zeros between 'p' and number (we do not want predicates which begin with the same sub-string, e.g. 'p1' and 'p11'; we use 'p01')
end
message = sprintf('\nLTL_x (without "next" operator) formula (between apostrophes) involving some of %d propositions (regions of partition) (use p1,p2,... as atomic propositions)',N_s);
%message = sprintf('%s\n%s',message,'Atomic propositions are of form p0...0x, where x is an integer; true, false');
message = sprintf('%s\n\t%s',message,'Boolean operators: ! - negation; & - and; | - or; -> - implication; <-> - equivalence');
message = sprintf('%s\n\t%s',message,'Temporal operators: U - until; R - release; F - eventually; G - always; X - next');
%message = sprintf('%s\n%s',message,'---example of formula: (F p1) & G !(p2 | p3)');
%message

formula = inputdlg(message,'Robot Motion Toolbox',1,{'(F u1) & G !(u2 | u3)'});
if isempty(formula)
    return;
end

for i=N_s:-1:1 %replace props px with p0...0x (start with the last one) - to not confund p1 with p1x, ...
    formula=regexprep(formula,['\<' new_alph{i} '\>'],alphabet{i});    %sintax \< , \> for exact match
end %formula=regexprep(formula,new_alph,alphabet(1:end-1));    %bad sintax for more than 10 props

