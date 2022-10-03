%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  draw_Trajectories
%   Eduardo Montijano
%   Date: March 2021
%   New drawing function to work for CASE 2021 + Robotarium
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Robots = rmt_draw_FinalTrajectories(Robots,T)

for i=1:length(Robots)    
    Tray=Robots{i}.Path;
    if ~isempty(Tray)
        trayx = zeros(1,length(Tray));
        trayy = zeros(1,length(Tray));
        for k = 1:length(Tray)
            trayx(k) = T.mid{Tray(k)}(1);
            trayy(k) = T.mid{Tray(k)}(2);
        end
    end
end
drawnow
end
