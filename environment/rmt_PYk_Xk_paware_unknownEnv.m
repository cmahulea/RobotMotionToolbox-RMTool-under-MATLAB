function PYX = rmt_PYk_Xk_paware_unknownEnv(dist, Xk, numRegions, prob)
    %CAMBIO: si esta a distancia 1 (es celda vecina)
    % entonces el robot sabe al 100% de qué color de celda se trata
    if dist <= 1
        pmax = prob;
    elseif dist <= 3
        pmax = prob/1.5;
    elseif (dist>3) && (dist<=6)
        pmax = prob/2;
    else
        pmax = 1/numRegions;
    end
    if (pmax < 1/numRegions)
        pmax = 1/numRegions;
    end

    PYX = (1-pmax)/(numRegions-1)*ones(numRegions,1);
    PYX(Xk) = pmax;
end