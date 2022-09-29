function PYX = PYk_Xk_paware(dist, Xk, numRegions)

% if Xk == numRegions-1 % Obstacle
    %CAMBIO: si esta a distancia 1 (es celda vecina)
    % entonces el robot sabe al 100% de qué color de celda se trata
    if dist <= 1
        pmax = 1;
    elseif dist < 3
        pmax = 0.5;
    elseif (dist>=3) && (dist<6)
        pmax = 0.3;
    else
        pmax = 1/numRegions;
    end
% else
%     if dist < 1
%         pmax = 0.8;
%     elseif (dist>=1) && (dist<2)
%         pmax = 0.6;
%     elseif (dist>=2) && (dist<3)
%         pmax = 0.4;
%     else
%         pmax = 1/numRegions;
%     end
% end

PYX = (1-pmax)/(numRegions-1)*ones(numRegions,1);
PYX(Xk) = pmax;