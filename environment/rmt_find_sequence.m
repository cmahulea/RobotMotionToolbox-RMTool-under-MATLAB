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

function [ret,Run_cells] = rmt_find_sequence(m0, vector, Pre, Post,steps, Run_cells)
ret = sprintf(' Step %d:',steps);
places = rmt_marking2places(m0);
% Run_cells = [Run_cells , places]; %initial are already in Run_cells from previous step
for i = 1 : length(places)-1
    for j = 1 : length(m0(places(i)))
        ret = sprintf('%s %d, ',ret,places(i));
    end
end
for j = 1 : length(m0(places(length(places))))-1
    ret = sprintf('%s %d, ',ret,places(length(places)));
end
ret = sprintf('%s %d ',ret,places(length(places)));

while (sum(vector)>0)
    tenabled = [];
    for i = 1 : size(Pre,2)
        if (min(m0 - Pre(:,i)) >= 0) 
            tenabled = [tenabled,i];
        end
    end
    tfire = intersect(tenabled,find(vector>0));
    C = Post - Pre;
    sigma = zeros(size(Pre,2),1);
    sigma(tfire(1)) = 1;
    mnew = m0 + (Post-Pre)*sigma;
    vector = vector - sigma;
    m0 = mnew;
    steps = steps + 1;
    ret = sprintf('%s \n Step %d:',ret,steps);
    
    % places = find(m0>0);
    places = rmt_marking2places(m0);
    Run_cells = [Run_cells , places];
    for i = 1 : length(places)-1
        for j = 1 : length(m0(places(i)))
            ret = sprintf('%s %d, ',ret,places(i));
        end
    end
    for j = 1 : length(m0(places(length(places))))-1
        ret = sprintf('%s %d, ',ret,places(length(places)));
    end
    ret = sprintf('%s %d ',ret,places(length(places)));
end