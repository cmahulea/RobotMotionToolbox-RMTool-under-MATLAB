%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  draw_Trajectories
%   Eduardo Montijano
%   Date: March 2021
%   New drawing function to work for CASE 2021 + Robotarium
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Robots = draw_FinalTrajectories(Robots,T,RobotColors)

for i=1:length(Robots)    
    Tray=Robots{i}.Path;
    if ~isempty(Tray)
        trayx = zeros(1,length(Tray));
        trayy = zeros(1,length(Tray));
        for k = 1:length(Tray)
            trayx(k) = T.mid{Tray(k)}(1);
            trayy(k) = T.mid{Tray(k)}(2);
        end
        % Estas 3 lineas de abajo dibujan las trayectorias de todos los
        % robots en la figura que se encuentre activa
        %plot(trayx, trayy, 'o-', 'LineWidth',4, 'Color', RobotColors(i,:),'MarkerFaceColor',RobotColors(i,:));
        %plot(trayx(end),trayy(end),'s', 'LineWidth',4,'MarkerSize',50,'Color',RobotColors(i,:));
        %plot(trayx(1),trayy(1),'o', 'LineWidth',4,'MarkerSize',50,'Color',RobotColors(i,:));
    end
end
drawnow
end
