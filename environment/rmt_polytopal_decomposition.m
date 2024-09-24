function [C,adj,varargout]=rmt_polytopal_decomposition(obstacles,env_bounds)

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
%   Second version released on September, 2024. 
%   Last modification September 24, 2024.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

%convert obstacles (all are convex hulls) in H-representation, and world - boundaries
n_ob=length(obstacles);
A=cell(1,n_ob+1);
b=cell(1,n_ob+1);
for i=1:length(obstacles)
    [A_i,b_i] = V2H_conversion(obstacles{i}); %H-representation
    A{i}=A_i;
    b{i}=-b_i;  %object i: Ax+b<=0;
end

A{end}=[0 -1;-1 0;0 1;1 0];    %H-repres for world (boundaries)
b{end}=[env_bounds(3) env_bounds(1) -env_bounds(4) -env_bounds(2)]';


%from here on, inspiration is from my function "tran_sys_polytope", used in LTL_control
Ain=cell2mat(A(1:end-1)');  %matrix with inequalities for all atomic prop.
Bin=cell2mat(b(1:end-1)');

n=size(Ain,2);  %space dimension
Nh=length(Bin); %number of hyperplanes defining obstacles
N_p=length(A)-1;    %number of atomic props in formula (without left-over space)

%if Nh is greater or around 20, Windows can stop Matlab (because the computation will take too long: 2^Nh possibilities)
%try to eliminate some infeasible combinations starting with a small n_h (will also allow us to find the number of states, for preallocating data)
n_h=min(5,Nh); %number of hyperplanes to start with
step_n=5;   %maximum step to increase n_h
x=min(step_n,(Nh-n_h));   %actual step to increase n_h
ind=0:(2^n_h-1);    %initial indices which can be feasible
while 1 %x>0 || isempty(who('new_ind'))    %iterate while Nh is not reached (force at least one iteration)
    new_ind=[];
    for i=ind   %for every feasible index from n_h hyperplanes, try any possible index from x
        for j=0:(2^x-1)
            index=j*2^n_h+i;    %index to be tested (start with right n_h bits, add x bits to left) (start from last (n_h+x) propositions (lines) from Ain, bin)
            s=double(dec2bin( index,(n_h+x) ))-double('0');   %signs for inequalities (1 means Ax+b>0, 0 means Ax+b<0 (>=,<=))
            signs=(-1).^s';   %column vector with signs (-/+) for inequality corresponding to each row (-1 means Ax+b>0, 1: Ax+b<0)
            H.A=[diag(signs)*Ain( (end+1-(n_h+x)):end ,:) ; A{end}];    %change signs of rows of Ain, Bin according to values in signs (for -1 change sign)
            H.B=-[signs.*Bin((end+1-(n_h+x)):end) ; b{end}]; %change sign for B (in cdd+, Ax<=B); H is a structure to call cdd+
            Spol.V=H2V_conversion(H.A,H.B)';    %create vertex representation of sub-polytope (if such polytope exists)
            if (size(Spol.V,1) > n) && (rank([Spol.V , ones( size(Spol.V,1), 1)]) == (n+1)) %first condition is much faster to evaluate, so use short-circuit operator (evaluate second only when first is true)
                new_ind=[new_ind, index];   %store indices of feasible polytopes
            end
        end
    end
    ind=unique(new_ind);    %update vector with indices (to cover last n_h+x hyperplanes, not only last n_h); sort vector
    n_h=n_h+x;  %update n_h
    x=min(step_n,(Nh-n_h));   %update x
    if x==0 %Nh was reached
        break
    end
end

%we have vector ind with values between 0 and 2^Nh-1, giving indices of all feasible subpolytopes
rem_i=[]; %indices of states that are contained in obstacles (we don;t need those in cell decomposition)

sp_no=length(ind);  %number of feasible states (polytopes)
Signs=ones(sp_no,Nh); %preallocate
C=cell(1,sp_no);  %each element will contain vertices of current state (polytope)
for i=1:sp_no   %feasibility was already checked for the binary combinations corresponding to elements in vector ind
    s=double(dec2bin(ind(i),Nh))-double('0');   %signs for inequalities (1 means Ax+b>0, 0 means Ax+b<0 (>=,<=))
    signs=(-1).^s';   %column vector with signs (-/+) for inequality corresponding to each row (-1 means Ax+b>0, 1: Ax+b<0)
    H.A=[diag(signs)*Ain ; A{end}];    %change signs of rows of Ain, Bin according to values in signs (for -1 change sign)
    H.B=-[signs.*Bin ; b{end}]; %change sign for B (in cdd+, Ax<=B); H is a structure to call cdd+
    Spol.V=H2V_conversion(H.A,H.B)';    %create vertex representation of sub-polytope (test for feasability is redundant, because feasible combinations were already found)
    
    C{i}=Spol.V';  %put vertices of new sub-pol in cell C (vertices on columns)
    Signs(i,:)=s; %retain s (giving signs) of each state (as rows of a matrix) - will be useful in finding adjacency

    %remove indices of states contaiend in obstacles
    for j=1:N_p   %for each proposition (without leftover space)
        if isempty( find( s(1:size(A{j},1)) , 1))  %all signs corresponding to prop j are 0, so prop j is satisfied by current polytope (i)
            rem_i = [rem_i, i]; %current state will be removed; continue with next
            break;
        end
        s(1:size(A{j},1))=[]; %eliminate first elements from signs vector (they are already verified)
    end
end

%update after removing states included in obstacles:
sp_no=sp_no-length(rem_i);  %updated number of feasible states (polytopes)
Signs(rem_i,:)=[];
C(rem_i)=[];  %this containes the cell decomposition (without polytopes splitting obstacles)

adj=sparse(eye(sp_no));  %adjacency matrix (symmetric)

if nargout>2    %if desired, compute middle points between adjacent cells (useful for an angular path finding); avoid reutrn a cell, because of memory usage:
    middle_X=sparse(sp_no,sp_no);   %element (i,j) is zero if i=j or if cells i and j are not adjacent, and otherwise it contains X-coordinate for middle segment between i and j
    middle_Y=sparse(sp_no,sp_no);
    if nargout>4    %return also common line segments shared by adjacent cells
        com_F=cell(sp_no,sp_no);    %com_F{i,j} is a matrix with vertices of line segment shared by adjacent cells i,j; vertices are on columns
    end
end

for i=1:(sp_no-1)
    %find adjacency for polytope i
    signs=Signs(i,:); %signature vector of state i
    %equal = (repmat(signs,sp_no,1) == Signs);  %useful for searching rows which differ than row i on only one position
    %adj(i, find(sum(equal,2) >= (Nh-1))) = 1;   %rows of equal whose sum is Nh-1 gives adjacent subpolytopes (only row i has sum Nh - loop in each state)
%     adj(i,i)=1;  %self-loop for each state
%     for j=setdiff(1:sp_no,i)    %j~=i; it's possible to have more more than one difference of sign between 2 adjacent states (see below) - if some props define the same hyperplane
    for j=(i+1):sp_no
        inequal = find(signs ~= Signs(j,:));  %propositions giving diferences between signs of states i and j
        if rank([Ain(inequal,:) Bin(inequal,:)])==1
            adj(i,j)=1;  %if there are more than one diferences, check (through rank) if props giving diferences are not identical or opposite
            adj(j,i)=1;
            
            if nargout>2
                %middle of segment between cells i and j:
                s_i=(-1).^(Signs(i,:)');
                s_j=(-1).^(Signs(j,:)');
                H.A=[diag(s_i)*Ain ; diag(s_j)*Ain ; A{end}]; %intersection of polytopes corresponding to states i and j
                H.B=-[s_i.*Bin ; s_j.*Bin ; b{end}];
                V.V=H2V_conversion(H.A,H.B)';    %create vertex representation of sub-polytope (test for feasability is redundant, because feasible combinations were already found)
                if size(V.V,1)==2
                    middle_temp=mean(V.V);
                    middle_X(i,j)=middle_temp(1);
                    middle_X(j,i)=middle_temp(1);
                    middle_Y(i,j)=middle_temp(2);
                    middle_Y(j,i)=middle_temp(2);
                    if nargout>4    %common line segments shared by adjacent cells
                        com_F{i,j}=sortrows(V.V)';  %com_F{i,j} has form [x_i' , x_i'' ; y_i' , y_i''], x_i'<=x_i''
                        com_F{j,i}=com_F{i,j};
                    end
                else
                    fprintf('\nError when finding middle point between cells %d and %d',i,j);
                end
            end
        end
    end
end
% adj(sp_no,sp_no)=1; %self-loop in last polytope

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
