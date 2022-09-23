function T = generateTransitionPN_robotarium(Height,Width)

C = Width*Height;
T.Q=1:C;
T.adj = sparse(C,C);
T.Vert = cell(1,C);
T.mid = cell(1,C);
t = 0;
for y = 1:Height-1
    for x = 1:Width-1
        t = t+1;
        T.adj(t,t+1) = 1;
        T.adj(t+1,t) = 1;
        T.adj(t,t+Width) = 1;
        T.adj(t+Width,t) = 1;
        T.mid{t} = 0.4*[x-0.5,y-0.5];
        T.Vert{t} = 0.4*[x-1,x,x,x-1;y-1,y-1,y,y];
    end
    x = Width;
    t = t+1;
    T.adj(t,t+Width) = 1;
    T.adj(t+Width,t) = 1;
    T.mid{t} = 0.4*[x-0.5,y-0.5];
    T.Vert{t} = 0.4*[x-1,x,x,x-1;y-1,y-1,y,y];
end
y = Height;
for x = 1:Width-1
    t = t+1;
    T.adj(t,t+1) = 1;
    T.adj(t+1,t) = 1;
    T.mid{t} = 0.4*[x-0.5,y-0.5];
    T.Vert{t} = 0.4*[x-1,x,x,x-1;y-1,y-1,y,y];
end
x = Width;
T.mid{end} = 0.4*[x-0.5,y-0.5];
T.Vert{end} = 0.4*[x-1,x,x,x-1;y-1,y-1,y,y];