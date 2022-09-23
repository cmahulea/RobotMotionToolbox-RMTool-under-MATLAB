function [C,adj,OBS_set,obs,varargout]=triangular_decomposition_regions(objects,env_bounds)
%mk, version: may 2012 (regions/objects may intersect or go outside environment)
%objects is a cell where each element is a matrix with 2 rows, giving the vertices of current region of interest
%OBS_set is a matrix having on rows the indices of satisfied regions, padded with zeros until number of regions
%only feasible conjunctions of regiosn are included in OBS_set, and the last row is for the free )leftover) space
%obs are the observables of each triangle as row indices in OBS_set (OBS_set(obs(k)) contains the regions satisfied by triangle k, padded with zeros)
%env_bounds has the form [x_min,x_max,y_min,y_max]

%first begin creating points (vertices) X
X = [env_bounds(1) env_bounds(3); %space boundary
    env_bounds(2) env_bounds(3);
    env_bounds(2) env_bounds(4);
    env_bounds(1) env_bounds(4)];

% X = [x_min y_min; %space boundary
%     x_max y_min;
%     x_max y_max;
%     x_min y_max];

%begin creating constraints Cst (indices of lines from X giving linear constraints)
Cst = [1 2; %space boundary
         2 3;
         3 4;
         4 1];
middle_reg = []; %centroids of the regions

%add obstacle information
for i=1:length(objects)
    ind=size(X,1)+1;    %index for points
    X=[ X; [objects{i}]' ];
    for j=1:(size(objects{i},2)-1)
        Cst=[ Cst; [ind+j-1 , ind+j ] ];  %object's vertices are arranged in order of parcurging convex hull
    end
    Cst=[ Cst; [ind+j , ind ] ];  %close current object
end

%constrained triangulation (Matlab 2010b); warnings may appear to indicate that constraining edges (obstacles) intersect, or some points are repeated
warning('off', 'all');
Triang = DelaunayTri(X, Cst);   %special structure
warning('on', 'all');

X=Triang.X; %new points (vertices) may have appeared, in case of obstacles intersecting
triangles=Triang.Triangulation;  %indices of triangles (points from nex X)

%from the returned triangles, some are inside regions (these are triangulated as well);
%we cannot have triangles partially overlapping with regions (becasue of Constrained Delaunay);
%however, new points (vertices) may appear - intersection of regions

%search indices of traingles outside of environment bounds
ind_out=[]; %feasible indices of triangles (row indices in triangles)
for k=1:size(triangles,1)
    centr=mean(X(triangles(k,:),:));   %centroid of triangle k (if centroid belongs to one or more obstacles, the whole triangle belongs to that/those obstacles)
    if centr(1)<env_bounds(1) || centr(2)<env_bounds(3) || centr(1)>env_bounds(2) || centr(2)>env_bounds(4) %current triangle outside of bounds (possible if obstacles cross outside)
        ind_out=[ind_out,k];    %triangle with index k will be removed
    else
        middle_reg{k} = centr;
    end
end
triangles(ind_out,:)=[];  %remove triangles outside bounds
middle_reg(ind_out)=[];
%find observations
nr_tri=size(triangles,1);  %all triangles are now feasible (regions are also split)
N_p=length(objects);    %number of regions
OBS_set=[]; %will be a matrix with N_p columns, giving on rows the possible observations. A row like [1 3 0...0] means that current observation is only regions 1 and 3
%OBS_set will be sorted, and last row will correspond to free space, being [(N_p+1) 0 ... 0]
obs=zeros(1,nr_tri); %observations of each cell, as indices for rows of OBS_set
Sat_reg=zeros(nr_tri , N_p); %on row k we store satisfied regs of triangle k
for k=1:nr_tri
    in_reg=zeros(1,length(objects));    %will be 1 for regions containing current triangle (may be multiple ones)
    centr=mean(X(triangles(k,:),:));   %centroid of triangle k (if centroid belongs to one or more obstacles, the whole triangle belongs to that/those obstacles)
    for i=1:length(objects) %for each obstacle
        if inpolygon(centr(1),centr(2),objects{i}(1,:),objects{i}(2,:))      %current triangle is inside region i
            in_reg(i)=1;
        end
    end
    satisf_reg=find(in_reg);    %satisfied regs, in form [1 3] if only regions 1 & 3 are satisfied
    if isempty(satisf_reg)   %no satisfied reg. (add a dummy prop and pad with zeros)
        satisf_reg=zeros(1,N_p);
        satisf_reg(1)=N_p+1;    %index for dummy region (free space)
    elseif length(satisf_reg)<N_p   %pad with zeros until number of regions
        satisf_reg((end+1):N_p)=0;
    end     %satisf_reg has N_p value, having form [1 3 0...0]
    Sat_reg(k,:)=satisf_reg;    %store observation (do not compute row index for OBS_set yet, because OBS_set wil be sorted)
    OBS_set=unique([OBS_set ; satisf_reg] , 'rows');    %add current observation (may already be there)  (unique([OBS_set ; satisf_reg] , 'rows') would sort and keep unique, but it may be slower reiterated)
end
OBS_set = unique(OBS_set,'rows');   %keep unique rows and sort them (OBS_set contains only possible overlappings of regions)
for k=1:nr_tri   %now find observable of each triangle
    [ignore,obs(k)]=ismember(Sat_reg(k,:),OBS_set,'rows');    %observable
end

%construct cell C (triangle vertices in each element) and adjacency adj
%return: C- containing cel vertices (cell with each element a matrix with 2 rows and 3 columns)
%    and adj - adjacency matrix, with adj(i,j)=1 if cell i is adjacent to j (adj(i,i)=1 and adj is symmetric)s
C=cell(1,nr_tri);
adj=sparse(eye(nr_tri)); %self transitions in every cell

if nargout>4    %if desired, compute middle points between adjacent cells (useful for an angular path finding); avoid reutrn a cell, because of memory usage:
    middle_X=sparse(nr_tri,nr_tri);   %element (i,j) is zero if i=j or if cells i and j are not adjacent, and otherwise it contains X-coordinate for middle segment between i and j
    middle_Y=sparse(nr_tri,nr_tri);
end

for i=1:nr_tri
    C{i}=(X(triangles(i,:),:))';    %in matrix triangle, the columns are indices of points from X composing the triangle
    for j=i+1:nr_tri %go only through higher indices (adjacency matrix is symmetric)
        common_v=intersect(triangles(i,:), triangles(j,:)); %indices of common vertices (nodes) of cells i and j
        if length(common_v)==2 %two common vertices means adjacency
            adj(i,j)=1;
            adj(j,i)=1; %symmetric
            
            if nargout>4
                %middle of segment between cells i and j:
                middle_temp=mean(X(common_v,:));
                middle_X(i,j)=middle_temp(1);
                middle_X(j,i)=middle_temp(1);
                middle_Y(i,j)=middle_temp(2);
                middle_Y(j,i)=middle_temp(2);
            end
        end
    end
end
% adj=sparse(adj);    %convert to sparse

%arrange vertices of each cell from decomposition to be convex hull
for i=1:nr_tri
    ch_in=convhull(C{i}(1,:),C{i}(2,:));
    C{i}=C{i}(:,ch_in(1:length(ch_in)-1));
end

if nargout>4
    %return middle of adjacent segments:
    varargout(1)={middle_X};
    varargout(2)={middle_Y};
    varargout(3)={middle_reg};
end
