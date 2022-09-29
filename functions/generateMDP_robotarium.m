function T = generateMDP_robotarium(Height,Width)

C = Width*Height;
T.Q=1:C;
T.Vert = cell(1,C);
T.mid = cell(1,C);
t = 0;
ntrans = 0;
pstay = 0.05;
pmoveok = 0.7;
pfail = 1 - pstay - pmoveok;
for y = 1:Height
    for x = 1:Width
        %% Action 'UP'
        if y+1 <= Height
            ntrans = ntrans+1;
            T.P(ntrans,:) = zeros(1,C);
            T.P(ntrans,Width*(y-1)+x)=pstay;
            T.P(ntrans,Width*y+x)=pmoveok;
            switch x
                case 1
                    T.P(ntrans,Width*y+x+1)=pfail;
                case Width
                    T.P(ntrans,Width*y+x-1)=pfail;
                otherwise
                    T.P(ntrans,Width*y+x+1)=pfail/2;
                    T.P(ntrans,Width*y+x-1)=pfail/2;
            end
        end
        
        %% Action 'DOWN'
        if y-1 > 0
            ntrans = ntrans+1;
            T.P(ntrans,:) = zeros(1,C);
            T.P(ntrans,Width*(y-1)+x)=pstay;
            T.P(ntrans,Width*(y-2)+x)=pmoveok;
            switch x
                case 1
                    T.P(ntrans,Width*(y-2)+x+1)=pfail;
                case Width
                    T.P(ntrans,Width*(y-2)+x-1)=pfail;
                otherwise
                    T.P(ntrans,Width*(y-2)+x+1)=pfail/2;
                    T.P(ntrans,Width*(y-2)+x-1)=pfail/2;
            end
        end
        
        %% Action 'RIGHT'
        if x+1 <= Width
            ntrans = ntrans+1;
            T.P(ntrans,:) = zeros(1,C);
            T.P(ntrans,Width*(y-1)+x)=pstay;
            T.P(ntrans,Width*(y-1)+x+1)=pmoveok;
            switch y
                case 1
                    T.P(ntrans,Width*y+x+1)=pfail;
                case Height
                    T.P(ntrans,Width*(y-2)+x+1)=pfail;
                otherwise
                    T.P(ntrans,Width*y+x+1)=pfail/2;
                    T.P(ntrans,Width*(y-2)+x+1)=pfail/2;
            end
        end
        
        %% Action 'LEFT'
        if x-1 > 0
            ntrans = ntrans+1;
            T.P(ntrans,:) = zeros(1,C);
            T.P(ntrans,Width*(y-1)+x)=pstay;
            T.P(ntrans,Width*(y-1)+x-1)=pmoveok;
            switch y
                case 1
                    T.P(ntrans,Width*y+x-1)=pfail;
                case Height
                    T.P(ntrans,Width*(y-2)+x-1)=pfail;
                otherwise
                    T.P(ntrans,Width*y+x-1)=pfail/2;
                    T.P(ntrans,Width*(y-2)+x-1)=pfail/2;
            end
        end
        t = t+1;
        T.mid{t} = 0.4*[x-0.5,y-0.5];
        T.Vert{t} = 0.4*[x-1,x,x,x-1;y-1,y-1,y,y];
    end
end