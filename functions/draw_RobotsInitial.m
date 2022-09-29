%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  draw_RobotsInitial
%   Eduardo Montijano
%   Date: March 2021
%   New drawing function to work for CASE 2021 + Robotarium
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Robots = draw_RobotsInitial(Robots,RobotColors)

for i=1:length(Robots)
    Robots{i}.plotPoseGlobal = plot(Robots{i}.pos(1),Robots{i}.pos(2),'ko', 'LineWidth',5,'MarkerSize',70,'Color',RobotColors(i,:));
%     Robots{i}.robotNumber = text(Robots{i}.pos(1)+0.1,Robots{i}.pos(2)+0.15,sprintf('r_{%d}',i),'HorizontalAlignment','center','Color','k','FontSize',36,'FontAngle','italic','FontName','TimesNewRoman');
%     Robots{i}.plotPoseGlobal = plot(Robots{i}.pos(1),Robots{i}.pos(2),'ko','MarkerSize',4,'MarkerFaceColor','b');
end
