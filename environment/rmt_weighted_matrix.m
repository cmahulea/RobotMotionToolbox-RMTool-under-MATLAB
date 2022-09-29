function W = rmt_weighted_matrix(G,weights)
global vnet;
N = size(G,1);
W = zeros(N,N);
if weights ==1
    %METROPOLIS WEIGHTS
    for i=1:N
        for j=i+1:N
            if G(i,j)
                Nj = max(sum(G(i,:)),sum(G(j,:)))+1;
                W(i,j) = 1/Nj;
                W(j,i) = 1/Nj;
            end
        end
        W(i,i) = 1-sum(W(i,:));
    end
elseif weights == 2
    %BEST CONSTANT WEIGHT
    G2 = zeros(N,N);
    for i=1:N
        G2(i,i) = sum(G(i,:));
    end
    E = eig(G2-G);
    alfa = 2/(E(2)+E(N));
    W = alfa*G + eye(N) - alfa*G2;
elseif weights == 3
    uno = ones(N,1);
    G2 = zeros(N,N);
    G3 = zeros(N,N);
    for i=1:N
        for j=i+1:N
            G2(i,j) = G(i,j);
            if G(i,j)
                G3(i,j) = 1.0/max(sum(G(i,:))+1,sum(G(j,:))+1);
            end
        end
%         G2(i,i) = 1.0;
        G3(i,i) = 1.0-sum(G3(i,:));
    end
    vnet=reshape(G2,1,N^2);
    vA=reshape(G3,1,N^2);
    v=vA(vnet>0);
    a=vector_to_matrix(v);
    aa=a-uno*uno'/N;
    b = max(abs(eig(aa)));
    %%%%%%%%
    % Optimal w, starting from v
    %%%%%%%%
    w = fminsearch(@spec_rad,v,optimset('Display','off'));%,'MaxIter',5*N));
    %%%%%%%%
    % Tranformation of w to matrix
    %%%%%%%%
    A=vector_to_matrix(w);
    AA=A-uno*uno'/N;
    b = max(abs(eig(AA)));
    W = A;
elseif weights == 4 %Row stochastic
    W = G+eye(N);
    for i=1:N
        W(i,:) = W(i,:)/sum(W(i,:));
    end
end

function rho = spec_rad(v)
    global vnet;
    N = sqrt(size(vnet,2));
    uno = ones(N,1);
    %
    %  Dado el vector v, se calcula la matriz de adyacencia a,
    A=vector_to_matrix(v);
    %  sele quita el valor propio 1 
    A=A-uno*uno'/N;
    %  y se calcula el radio espectral
    rho = max(abs(eig(A)));

function A=vector_to_matrix(v)
%   Transform the vector v to a valid adjacency matrix
    %
    %  va es la matriz extendida como vector
    %
    global vnet;
    N = sqrt(size(vnet,2));
    uno = ones(N,1);
    va = zeros(1,N^2);
    va(vnet>0)=v;
    %  se convierte a matriz
    A=reshape(va,N,N);
    %  la parte inferior se rellena
    A=A+A';
    % se añade la diagonal
    A=A+diag(uno-A*uno);
