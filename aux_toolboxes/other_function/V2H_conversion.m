function [A,b] = V2H_conversion(V)
%converts convex and bounded poligons from V- to H- representation in 2D
%Input: matrix V, with 2 rows; each column contains the coordinates of one vertex.
% The vertices from V should not necessarily be sorted in a specific order
%Outputs: matrix A (with 2 columns) and column vector b, s.t. the poligon
% is described by any [x;y] that satisfies A*[x;y] <= b.

if size(V,1)~=2
    fprintf('\nError - input matrix should have 2 rows, and at least one column!\n');
    return
end

digits = 10;  %precision for minimizing effect of representation errors
% prec = 10^-digits;
V = round(V,digits); %or V = round(V/prec)*prec;

try  %i convhull works (at least 3 non-colinear vertices
    k = convhull(V(1,:),V(2,:));
    V=V(:,k);   %order vertices, and duplicate first with last vertex
    centroid = mean(V(:,1:end-1),2);
    [~,n] = size(V);    %number of rows is 2
    A=zeros(n-1,2);
    b=zeros(n-1,1);
    for i=1:n-1 %take each pair of successive vertices
        A(i,1) = V(2,i+1)-V(2,i);
        A(i,2) = V(1,i)-V(1,i+1);
        b(i) = V(1,i)*V(2,i+1)-V(2,i)*V(1,i+1);
        
        if ~ (A(i,:)*centroid <= b(i))  %this test is redundant due to order given by convhull function
            fprintf('\n Wrong inequality - %g!\n',i);
        end
    end
catch
     V = unique(V','rows')';  %eliminate duplicate vertices, if any
     A=zeros(4,2);
     b=zeros(4,1);
     if size(V,2)>=2    %2 or more collinear vertices
         A(1,1) = V(2,end)-V(2,1);  %from unique, vertices are sorted based on x and then y
         A(1,2) = V(1,1)-V(1,end);
         b(1) = V(1,1)*V(2,end)-V(2,1)*V(1,end);
         A(2,:) = -A(1,:);  %change dirrection of above inequality (line)
         b(2) = -b(1);
         if V(1,1)~=V(1,end)    %different x coordinates, V(1,1) is minimum and V(1,end) maximum
             A(3,:) = [-1 0];   %x >= x_minimum
             b(3) = -V(1,1);
             A(4,:) = [1 0];   %x <= x_maximum
             b(4) = V(1,end);             
         else %same x coordinates => y's are different due to unique, also sorted
             A(3,:) = [0 -1];   %y >= y_minimum
             b(3) = -V(2,1);
             A(4,:) = [0 1];   %y <= y_maximum
             b(4) = V(2,end);               
         end
     else   %one vertex
         A(1,:) = [-1 0];   %x >= x_minimum
         b(1) = -V(1,1);
         A(2,:) = [1 0];   %x <= x_maximum
         b(2) = V(1,end);
         A(3,:) = [0 -1];   %y >= y_minimum
         b(3) = -V(2,1);
         A(4,:) = [0 1];   %y <= y_maximum
         b(4) = V(2,end);
     end
end

A = round(A,digits); %round results to prec
b = round(b,digits);
