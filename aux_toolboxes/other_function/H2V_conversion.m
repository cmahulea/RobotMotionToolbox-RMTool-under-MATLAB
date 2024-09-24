function V = H2V_conversion(A,b)
%converts convex and bounded poligons from H- to V- representation in 2D
%Inputs: matrix A (with 2 columns) and column vector b, s.t. the poligon
% is described by any [x;y] that satisfies A*[x;y] <= b.
%Output: matrix V, with 2 rows; each column contains the coordinates of one vertex.
% The vertices from V are not sorted in a specific sense; use convhull for sorting them.

if size(A,2)~=2 || size(b,2)~=1 || size(A,1)~=size(b,1)
    fprintf('\nError - incorrect size(s) of A and/or b!\n');
    return
end

digits = 10;  %precision for minimizing effect of representation errors
prec = 10^-digits;
A = round(A,digits);
b = round(b,digits);

n = size(A,1);    %number of inequalities

V = [];   %start with empty matrix
for i=1:n-1 %take each 2 different inequalities as equalities
    for j=i+1:n
        if abs(det(A([i,j],:))) > 1e-14    %non-singular matrix
            vertex = A([i,j],:) \ b([i,j],1);    %intersection of lines i and j (or vertex = inv(A([i,j],:))*b([i,j],1);)            
            if sum(A*vertex-b > prec)==0    %enforce a positive precision, due to representation errors
                vertex = round(vertex,digits);   %round result to prec
                V = [V, vertex];
%                 fprintf('\nVertex number %g, for inequalities %g and %g, was considered feasible within 1e-14 bounds.\n',size(V,2),i,j);
            end %else, at least one inequality is violated
        end  %no vertex for parallel or identical lines
    end
end
V = unique(V','rows')';  %eliminate duplicate vertices, if any inequalities were redundant
%if V remains empty => P is empty (non-feasible inequalities)
