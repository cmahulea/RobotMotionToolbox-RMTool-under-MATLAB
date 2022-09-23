function [G, Pos] = NetworkRandomPositions(N,Side,CommRadius,minDist)

if nargin~=4
    minDist=0;
end

Pos(1,1) = 0;
Pos(1,2) = 0;
for i=2:N
    cont = 0;
    while ~cont
        xpos = Side*(rand-0.5);
        ypos = Side*(rand-0.5);
        disok=1;
        neigok = 0;
        for j=1:i-1
            dist = (xpos-Pos(j,1))*(xpos-Pos(j,1))+(ypos-Pos(j,2))*(ypos-Pos(j,2));
            dist = sqrt(dist);
            if dist <= minDist
                disok=0;
            end
            if dist > minDist && dist <= CommRadius
                neigok=1;
            end
        end
        if disok && (neigok || i==1)
            Pos(i,1) = xpos;
            Pos(i,2) = ypos;
            cont=1;
        end
    end
end

G = eye(N);
for i=1:N
    for j=i+1:N
        dist = (Pos(i,1)-Pos(j,1))*(Pos(i,1)-Pos(j,1))+(Pos(i,2)-Pos(j,2))*(Pos(i,2)-Pos(j,2));
        dist = sqrt(dist);
        if dist <= CommRadius
            G(i,j) = 1;
            G(j,i) = 1;
        end
    end
end
