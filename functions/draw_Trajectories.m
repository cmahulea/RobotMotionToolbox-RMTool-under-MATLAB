%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  draw_Trajectories
%   Eduardo Montijano
%   Date: March 2021
%   New drawing function to work for CASE 2021 + Robotarium
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Robots = draw_Trajectories(Robots,T,RobotColors,multiPlot)

if nargin < 4
    multiPlot = 1;
end

if multiPlot
    NumSubplots = ceil(sqrt(length(Robots))); %For plot purposes
    % zzz = figure(2);
    % marker_size = determine_marker_size_bel(zzz, 0.2);
    for i=1:length(Robots)
        %figure(11)
        %subplot(NumSubplots,NumSubplots,i);
        
        for j=1:length(Robots)
            delete(Robots{i}.plotTrajectories{j})
            delete(Robots{i}.plotGoal{j})
            Tray=Robots{i}.PlannedTrajectories{j}.tray;
            if ~isempty(Tray)
                trayx = zeros(1,length(Tray));
                trayy = zeros(1,length(Tray));
                for k = 1:length(Tray)
                    trayx(k) = T.mid{Tray(k)}(1);
                    trayy(k) = T.mid{Tray(k)}(2);
                end
                % Estas 2 lineas de abajo dibujan la trayectoria en la
                % figura que esté activa (belief map 3)
                %Robots{i}.plotTrajectories{j} = plot(trayx, trayy, 'o--', 'LineWidth',2, 'Color', RobotColors(j,:),'MarkerFaceColor',RobotColors(j,:));
                %Robots{i}.plotGoal{j} = plot(trayx(end),trayy(end),'s', 'LineWidth',2,'MarkerSize',30,'Color',RobotColors(j,:));
            end
            Robots{i}.plotStart{j}.XData = T.mid{Robots{i}.TeamEstimatedPoses(j)}(1);
            Robots{i}.plotStart{j}.YData = T.mid{Robots{i}.TeamEstimatedPoses(j)}(2);
        end
        
        %drawnow
    end
else
    i = 1;
    for j=1:length(Robots)
        delete(Robots{i}.plotTrajectories{j})
        delete(Robots{i}.plotGoal{j})
        Tray=Robots{i}.PlannedTrajectories{j}.tray;
        if ~isempty(Tray)
            trayx = zeros(1,length(Tray));
            trayy = zeros(1,length(Tray));
            for k = 1:length(Tray)
                trayx(k) = T.mid{Tray(k)}(1);
                trayy(k) = T.mid{Tray(k)}(2);
            end
            Robots{i}.plotTrajectories{j} = plot(trayx, trayy, 'o--', 'LineWidth',8, 'Color', RobotColors(j,:),'MarkerFaceColor',RobotColors(j,:));
            Robots{i}.plotGoal{j} = plot(trayx(end),trayy(end),'s', 'LineWidth',8,'MarkerSize',50,'Color',RobotColors(j,:));
            Robots{i}.plotGoal{j}.MarkerSize = 50;
        end
        Robots{i}.plotStart{j}.XData = T.mid{Robots{i}.TeamEstimatedPoses(j)}(1);
        Robots{i}.plotStart{j}.YData = T.mid{Robots{i}.TeamEstimatedPoses(j)}(2);
        Robots{i}.plotStart{j}.MarkerSize = 50;
    end
end
