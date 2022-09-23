function PYX = rmt_PYk_Xk(dist, Xk, numRegions)

if dist < 1
    pmax = 1;
elseif (dist>=1) && (dist<2)
    pmax = 1;
elseif (dist>=2) && (dist<3)
    pmax = 0.5;
else
    pmax = 1/numRegions;
end

PYX = (1-pmax)/(numRegions-1)*ones(numRegions,1);
PYX(Xk) = pmax;