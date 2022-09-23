function Robots = updateDrawBeliefMaps(Robots,T,RegionColors)

NumSubplots = ceil(sqrt(length(Robots))); %For plot purposes
for i=1:length(Robots)
%     subplot(NumSubplots,NumSubplots,i);
%     eval(sprintf('subplot(%d,%d,%d)',length(Robots),1,i));
    eval(sprintf('subplot(%d,%d,%d)',NumSubplots,NumSubplots,i));
%     title(sprintf('Belief map of robot %d',i));
%     axis off;
    hold on;
    for x=1:length(T.Q)
        maxim_indices = find(Robots{i}.Beliefs{x}.Prob==max(Robots{i}.Beliefs{x}.Prob));
        Region = maxim_indices(ceil(rand*length(maxim_indices)));
        Robots{i}.Beliefs{x}.Region = Region;
        Robots{i}.BeliefsReg(x) = Region(1);
        Robots{i}.Beliefs{x}.filledRegion.FaceColor= RegionColors(Robots{i}.Beliefs{x}.Region,:);
    end
    Robots{i}.filledPoseLocal.XData = Robots{i}.x;
    Robots{i}.filledPoseLocal.YData = Robots{i}.y;
%     drawRobots(Robots(i));
end