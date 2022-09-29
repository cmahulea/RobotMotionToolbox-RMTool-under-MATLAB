function PYX = PYk_Xk_robotarium(dist, Xk, numRegions)

if dist < 0.4
    pmax = 0.85;
elseif (dist>=0.4) && (dist<0.6)
    pmax = 0.65;
else
    pmax = 1/numRegions;
end

PYX = (1-pmax)/(numRegions-1)*ones(numRegions,1);
PYX(Xk) = pmax;