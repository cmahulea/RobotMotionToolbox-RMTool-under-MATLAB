%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  draw_BeliefMapsUpdate
%   Eduardo Montijano
%   Date: March 2021
%   New drawing function to work for CASE 2021 + Robotarium
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Robots = rmt_draw_BeliefMapsUpdate(Robots,T,RegionColors,multiPlot)

if nargin < 4
    multiPlot = 1;
end

if multiPlot
    NumSubplots = ceil(sqrt(length(Robots))); %For plot purposes
    for i=1:length(Robots)
        %figure(10)
        %subplot(NumSubplots,NumSubplots,i);
        for x=1:length(T.Q)
            Robots{i}.plotBeliefs{x}.FaceColor = RegionColors(Robots{i}.Beliefs{x}.Region,:);
        end
        %     Robots{i}.plotPoseLocal.XData = Robots{i}.pos(1);
        %     Robots{i}.plotPoseLocal.YData = Robots{i}.pos(2);
        %drawnow
    end
else
    i = 1;
    for x=1:length(T.Q)
        Robots{i}.plotBeliefs{x}.FaceColor = RegionColors(Robots{i}.Beliefs{x}.Region,:);
    end
end
