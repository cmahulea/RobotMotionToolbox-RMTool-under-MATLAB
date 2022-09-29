

N = 5;
NClasses = 3;
Side = 8;
CommRad = 2;
minDist = 1;
[G, Pos] = NetworkRandomPositions(N,Side,CommRad,minDist);
W = WeightedMatrix(G,1);


pi0 = rand(NClasses,N);
for i=1:N
    pi0(:,i) = pi0(:,i)/sum(pi0(:,i));
end
pistar = ones(NClasses,1);
for i=1:N
    pistar = pistar.*pi0(:,i).^(1/N);
end
pistar = (pistar.^N)/sum(pistar.^N);
pik = pi0;
for it=1:100
    piN = ones(NClasses,N);
    for i=1:N
        for j=1:N
            if G(i,j)
                piN(:,i)=piN(:,i).*pik(:,j).^W(i,j);
            end
        end
    end
    pik = piN;
end
for i=1:N
    pik(:,i) = (pik(:,i).^N)/sum(pik(:,i).^N);
end
[pistar,pik]