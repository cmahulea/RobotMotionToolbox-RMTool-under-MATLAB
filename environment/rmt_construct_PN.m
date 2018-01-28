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

function [Pre,Post] = rmt_construct_PN(Adj)
% Construction of the Petri net
% This function returns Pre and Post matrixes relative to the created system
% Adj parameter represents the adjancecy matrix 

Post= zeros(size(Adj,1),[]);
Pre = zeros(size(Adj,1),[]);

for i = 1 : size(Adj,1)
    temp = setdiff(find(Adj(i,:)),i);
    for j = 1 : length(temp)
        % add a transition from place i to temp(j)
        % fprintf(1,'Add a transition from place p%d to p%d\n',i, temp(j));
        Pre = [Pre zeros(size(Pre,1),1)];
        Post = [Post zeros(size(Post,1),1)];
        Pre(i,size(Pre,2)) = 1;
        Post(temp(j),size(Post,2))=1;
    end
end

return
