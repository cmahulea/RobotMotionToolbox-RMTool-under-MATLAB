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


function [C,adj,varargout]=rmt_rectangular_decomposition(objects,env_bounds,varargin)
%mk, version: may 2012 (obstacles may intersect or go outside environment)
%rectangular decomposition, using quad-tree idea
%env_bounds has the form [x_min,x_max,y_min,y_max]
%prec is the max number of rectangle on each axis (power of 2); if not specified, it is 32 by default

if nargin==2    %precision was not specified
    prec=32;
else
    prec=varargin{1};
end

warning('off', 'MATLAB:mex:deprecatedExtension');

%convert obstacles (all are convex hulls) in H-representation
n_ob=length(objects);
A=cell(n_ob,1);
b=cell(n_ob,1);
for i=1:length(objects)
    M=objects{i}';
    H=cddmex('hull',struct('V',M)); %H-representation
    A{i}=H.A;
    b{i}=-H.B;  %object i: Ax+b<=0;
end

prec_x=(env_bounds(2)-env_bounds(1))/prec;  %minimum size on x-axis
prec_y=(env_bounds(4)-env_bounds(3))/prec;  %minimum size on y-axis

% %limits for world:
% x1=x_min;
% x2=x_max;
% y1=y_min;
% y2=y_max;
C={};    %start with empty cell for decomposition
C = split_check_rectangle(C, A, b, prec_x, prec_y, env_bounds(1), env_bounds(2), env_bounds(3), env_bounds(4)); %start with whole world, which will be recursively split in subrectangles

%find adjacency, based on coord
k=length(C);    %number of rectangles
adj=sparse(eye(k));

if nargout>2    %if desired, compute middle points between adjacent cells (useful for an angular path finding); avoid reutrn a cell, because of memory usage:
    middle_X=sparse(k,k);   %element (i,j) is zero if i=j or if cells i and j are not adjacent, and otherwise it contains X-coordinate for middle segment between i and j
    middle_Y=sparse(k,k);
    if nargout>4    %return also common line segments shared by adjacent cells
        com_F=cell(k,k);    %com_F{i,j} is a matrix with vertices of line segment shared by adjacent cells i,j; vertices are on columns
    end
end

Ak=[0 -1;-1 0;0 1;1 0]; %matrix A for H-repres of each rectangle
for i=1:(k-1)
    bi=[C{i}(2,1) C{i}(1,1) -C{i}(2,3) -C{i}(1,2)]';    %H-repres of rectangle i (y1,x1,-y2,-x2) - based on teh way vertices were added in cell C
    for j=(i+1):k
        bj=[C{j}(2,1) C{j}(1,1) -C{j}(2,3) -C{j}(1,2)]';    %H-repres of rectangle j
        V=cddmex('extreme',struct('A',[Ak;Ak],'B',[-bi;-bj]));
        if size(V.V,1)==2   %rectangles i and j are adjacent iff their intersection is a line segment (2 vertces)
            adj(i,j)=1;
            adj(j,i)=1;

            if nargout>2
                %middle of segment between cells i and j:
                middle_temp=mean(V.V);
                middle_X(i,j)=middle_temp(1);
                middle_X(j,i)=middle_temp(1);
                middle_Y(i,j)=middle_temp(2);
                middle_Y(j,i)=middle_temp(2);
                if nargout>4    %common line segments shared by adjacent cells
                    com_F{i,j}=sortrows(V.V)';  %com_F{i,j} has form [x_i' , x_i'' ; y_i' , y_i''], x_i'<=x_i''
                    com_F{j,i}=com_F{i,j};
                end
            end
        end
    end
end
% adj=sparse(adj);

%arrange vertices of each cell from decomposition to be convex hull
for i=1:length(C)
    ch_in=convhull(C{i}(1,:),C{i}(2,:));
    C{i}=C{i}(:,ch_in(1:length(ch_in)-1));
end

if nargout>2
    %return middle of adjacent segments:
    varargout(1)={middle_X};
    varargout(2)={middle_Y};
    %common line segments shared by adjacent cells
    if nargout>4
        varargout(3)={com_F};
    end
end

warning('on', 'MATLAB:mex:deprecatedExtension');


function C = split_check_rectangle(C, A, b, prec_x, prec_y, x1,x2,y1,y2)
%check if current rectalgle is free, occupied or mixed; if free->add it to cell decomposition, if mixed reccursively split
%argumets: C is currnet cell decomposition, x1,x2,y1,y2 limits of current rectangle, prec_x,prec_y minimum size of x or y

%Current rectangle: (first is world):
% M=[x1 y1; x2 y1; x2 y2; x1 y2]; %world vertices
% H=cddmex('hull',struct('V',M)); %H-representation
% R_A=H.A;  %world (current rectangle) in H-representation info
% R_b=-H.B;
R_A=[0 -1;-1 0;0 1;1 0];    %avoid the commented computations
R_b=[y1 x1 -y2 -x2]';

if (x2-x1)<prec_x || (y2-y1)<prec_y
    return; %precision reached for current sub-splitting
end

free=1; %assume rectangle R is free
area_int=0; %area of intersections of current rectangle with obstacles
for i=1:length(A);
    V=cddmex('extreme',struct('A',[A{i};R_A],'B',[-b{i};-R_b]));
    if (size(V.V,1) > 2) && (rank([V.V , ones( size(V.V,1), 1)]) == 3) %intersection is a full polytope
        free=0; %rectangle is certainly not free; it may be mixed or occupied
        [ignore, area]=convhull(V.V(:,1),V.V(:,2)); %area of current intersection
        area_int = area_int+area;
    end
end
if free==1  %rectangle is free, add its vertices to cell decomposition
    C{end+1}=[x1 x2 x2 x1;y1 y1 y2 y2];
elseif abs(area_int-(x2-x1)*(y2-y1)) >= (prec_x*prec_y/100)  %rectangle is mixed or occupied; decide which one after area of intersection
    %mixed rectangle, must be split in 4:
    C = split_check_rectangle(C,A,b,prec_x,prec_y, x1, (x1+x2)/2, y1, (y1+y2)/2); %lower-left rectangle
    C = split_check_rectangle(C,A,b,prec_x,prec_y, (x1+x2)/2, x2, y1, (y1+y2)/2); %lower-right rectangle
    C = split_check_rectangle(C,A,b,prec_x,prec_y, (x1+x2)/2, x2, (y1+y2)/2, y2); %upper-right rectangle
    C = split_check_rectangle(C,A,b,prec_x,prec_y, x1, (x1+x2)/2, (y1+y2)/2, y2); %upper-left rectangle
    
    %if rectangle is occupied (areas almost equal), do nothing - no need for else
end
