function G = rmt_computeCommGraph(Robots,CommRadius)

N_r = length(Robots);
G = eye(N_r,N_r);
for i=1:N_r
    for j=i+1:N_r
        if norm(Robots{i}.pos-Robots{j}.pos) <= CommRadius
            G(i,j) = 1;
            G(j,i) = 1;
        end
    end
end