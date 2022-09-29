function Robots = drawInitialBeliefMaps(Robots,T,RegionColors)

NumSubplots = ceil(sqrt(length(Robots))); %For plot purposes
for i=1:length(Robots)
    subplot(NumSubplots,NumSubplots,i);
%     eval(sprintf('subplot(%d,%d,%d)',length(Robots),1,i));
    title(sprintf('Belief map of robot %d',i));
    axis off;
    hold on;
    for x=1:length(T.Q)
%         if (max(Robots{i}.Beliefs{x}.Prob) == Robots{i}.Beliefs{x}.Prob(length(T.props)+1))%if more regions have the same probability including the empty observation (free space), consider the free space
%             Region = length(T.props)+1;
%         else
%             [~,Region]= max(Robots{i}.Beliefs{x}.Prob);
%         end
        maxim_indices = find(Robots{i}.Beliefs{x}.Prob==max(Robots{i}.Beliefs{x}.Prob));
        Region = maxim_indices(ceil(rand*length(maxim_indices)));
        Robots{i}.Beliefs{x}.Region = Region;
%         Robots{i}.BeliefsReg(x) = Region(1);
        Robots{i}.Beliefs{x}.filledRegion= fill(T.Vert{x}(1,:),T.Vert{x}(2,:),RegionColors(Region(1),:));
    end
    Robots{i}.filledPoseLocal = plot(Robots{i}.x,Robots{i}.y,'ko','MarkerSize',4,'MarkerFaceColor','b');
%     drawRobots(Robots(i));
end
