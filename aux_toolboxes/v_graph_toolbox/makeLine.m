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

function [Aeq Beq lb ub] = makeLine(point1, point2)
%point1 = [xa ya];
%point2 = [xb yb];
xa=point1(1,1);
ya=point1(1,2);
xb=point2(1,1);
yb=point2(1,2);
Aeq = [(yb-ya) (xa-xb)];
Beq = xb*(ya-yb)-yb*(xa-xb);
lb=[min(xa, xb); min(ya, yb)];
ub=[max(xa, xb); max(ya, yb)];
end

