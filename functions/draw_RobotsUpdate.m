%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  draw_RobotsUpdate
%   Eduardo Montijano
%   Date: March 2021
%   New drawing function to work for CASE 2021 + Robotarium
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Robots = draw_RobotsUpdate(Robots)

for i=1:length(Robots)
%     Robots{i}.plotText.Position = [Robots{i}.x-0.7,Robots{i}.y-0.7];
    Robots{i}.plotPoseGlobal.XData = Robots{i}.pos(1);
    Robots{i}.plotPoseGlobal.YData = Robots{i}.pos(2);
end
drawnow