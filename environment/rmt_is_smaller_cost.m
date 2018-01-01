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

function result = rmt_is_smaller_cost(candidate_cost,old_cost)
%ordering relation for comparing two row vectors:
%candidate is smaller than old if (1) there exist a position i such that candidate(i)<old(i) (strictly smaller (better)) and (2) for all previous positions candidate(i)<=old(i) (not larger/worse)
%result is -1 if candidate is not smaller, 0 if vectors are equal, 1 if candidate is (strictly) smaller
%i.e., start with first element, if position in candidate is smaller (better), the whole vector is better, if equal go on, if larger stop (candidate is larger(worse))

% %check sizes if desired (we comment it)
% if size(candidate_cost,1)~=1    %should have row vectors
%     candidate_cost=candidate_cost';
% end
% if size(old_cost,1)~=1    %should have row vectors
%     old_cost=old_cost';
% end
% if size(candidate_cost,1)~=1 || size(old_cost,1)~=1 || length(candidate_cost)~=length(old_cost)
%     fprintf('\n Error "is_smaller_cost": wrong size of input vectors (should be row and equal length vectors)!\n')
% end

if isequal(candidate_cost,old_cost) %rule out equality 
    result=0;
    return;
end

[~,ind]=sortrows([candidate_cost;old_cost]);
if ind(1)==1    %candidate is smaller
    result=1;
    return;
else    %equality was ruled out
    result=-1;
    return;
end

% %version below (commented) may be slower than the above one
% result=0;   %a flag; assume equal
% for i=1:length(candidate_cost)     %take each element, in their order
%     if candidate_cost(i) < old_cost(i)  %until now candidate was equal (otherwise function have been exited), now it's decided smaller
%         result=1;
%         return;
%     elseif candidate_cost(i) > old_cost(i)  %until now candidate was equal (otherwise function have been exited), now it's decided larger
%         result=-1;
%         return;
%     end
% end
