%    This is part of RMTool - Robot Motion Toolbox, for visibility graph functionality.
%
%    Copyright (C) 2016 RMTool developing team, with support of Silvia Magdici. For people, details and citing 
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

function index = returnIndex( pct, mult )
    index = 0;
    for k=1:length(mult)
        if norm(pct - mult{k})<=1e-10
            index=k;
            break
        end
    end
end

