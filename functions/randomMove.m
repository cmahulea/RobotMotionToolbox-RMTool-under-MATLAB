function Robots = randomMove(Robots,T,i)
%compute a random move for robot i to an adyacent region not occupied by other robot

adj_reg = find(T.adj(Robots{i}.reg,:)); %adyacent regions of robot i
for j = 1:length(Robots)
%     if (i == j)
%         continue;
%     end
    adj_reg = setdiff(adj_reg,Robots{j}.reg); %remove the adyacent regions occupied by other robots
    % HAY QUE QUITAR TAMBIEN LAS REGIONES QUE SEAN OBSTACULOS
    numCeldaObstaculo = max(T.obs) - 1;
    adj_reg = setdiff(adj_reg,find(T.obs == numCeldaObstaculo)); %MODIFICADO
end
%CAMBIO: si no hay region libre a donde ir (está encerrado)
% no moverse (quedarse en la misma celda)
if (isempty(adj_reg))
    adj_reg = Robots{i}.reg;
    move = 1;
else
    move = ceil(rand*length(adj_reg));
end
while Robots{i}.Beliefs{adj_reg(move)}.Prob(numel(T.props))>0.4
    move = ceil(rand*length(adj_reg));
end
fprintf(1,'\nRobot %d from %d moves to %d',i,Robots{i}.reg,adj_reg(move));
Robots{i}.PlannedTrajectories{i}.tray = [Robots{i}.reg,adj_reg(move)];
% Robots{i}.reg = adj_reg(move);
% Robots{i}.pos = T.mid{adj_reg(move)};


% Robots{i}.x = T.mid{adj_reg(move)}(1);
% Robots{i}.y = T.mid{adj_reg(move)}(2);
% Robots{i}.EstimatedReg(i) = Robots{i}.reg;
