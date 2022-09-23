%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  draw_BeliefMapsInitial
%   Eduardo Montijano
%   Date: March 2021
%   New drawing function to work for CASE 2021 + Robotarium
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Robots = draw_BeliefMapsInitial(Robots,T,RegionColors,RobotColors,multiPlot)

if nargin < 5
    multiPlot = 1;
end

if multiPlot
    NumSubplots = ceil(sqrt(length(Robots))); %For plot purposes
    for i=1:length(Robots)
        subplot(NumSubplots,NumSubplots,i);
        %     eval(sprintf('subplot(%d,%d,%d)',length(Robots),1,i));
        title(sprintf('Belief map of robot %d',i));
        axis off;
        hold on;
        for x=1:length(T.Q)
            if (max(Robots{i}.Beliefs{x}.Prob) == Robots{i}.Beliefs{x}.Prob(length(T.props)+1))%if more regions have the same probability including the empty observation (free space), consider the free space
                Region = length(T.props)+1;
            else
                [~,Region]= max(Robots{i}.Beliefs{x}.Prob);
            end
            %         maxim_indices = find(Robots{i}.Beliefs{x}.Prob==max(Robots{i}.Beliefs{x}.Prob));
            %         Region = maxim_indices(ceil(rand*length(maxim_indices)));
            Robots{i}.Beliefs{x}.Region = Region;
            Robots{i}.plotBeliefs{x} = fill(T.Vert{x}(1,:),T.Vert{x}(2,:),RegionColors(Region(1),:));
        end
        %     Robots{i}.plotPoseLocal = plot(Robots{i}.pos(1),Robots{i}.pos(2),'ko','MarkerSize',4,'MarkerFaceColor','b');
        for j=1:length(Robots)
            Robots{i}.plotTrajectories{j} = [];
            Robots{i}.plotGoal{j} = [];
            Robots{i}.plotStart{j} = plot(T.mid{Robots{i}.TeamEstimatedPoses(j)}(1),T.mid{Robots{i}.TeamEstimatedPoses(j)}(2),'o', 'LineWidth',2,'MarkerSize',30,'Color',RobotColors(j,:));
        end
        Robots{i}.plotStart{i}.MarkerFaceColor=RobotColors(i,:);
        %     drawRobots(Robots(i));
    end
else
    i=1;
    title(sprintf('Planning'));
    axis off;
    hold on;
    for x=1:length(T.Q)
        if (max(Robots{i}.Beliefs{x}.Prob) == Robots{i}.Beliefs{x}.Prob(length(T.props)+1))%if more regions have the same probability including the empty observation (free space), consider the free space
            Region = length(T.props)+1;
        else
            [~,Region]= max(Robots{i}.Beliefs{x}.Prob);
        end
        Robots{i}.Beliefs{x}.Region = Region;
        Robots{i}.plotBeliefs{x} = fill(T.Vert{x}(1,:),T.Vert{x}(2,:),RegionColors(Region(1),:));
        if Region==1 && Robots{i}.Beliefs{x}.Prob(1) < 0.65
            Robots{i}.plotBeliefs{x}.FaceColor = [1 1 1]-Robots{i}.Beliefs{x}.Prob(1);
        end
        %         Robots{i}.plotBeliefs{x}.EdgeColor=RegionColors(Region(1),:);
    end
    for j=1:length(Robots)
        Robots{i}.plotTrajectories{j} = [];
        Robots{i}.plotGoal{j} = [];
        Robots{i}.plotStart{j} = plot(T.mid{Robots{i}.TeamEstimatedPoses(j)}(1),T.mid{Robots{i}.TeamEstimatedPoses(j)}(2),'o', 'LineWidth',2,'MarkerSize',30,'Color',RobotColors(j,:));
        Robots{i}.plotStart{j}.MarkerFaceColor=RobotColors(j,:);
        for x=1:length(T.Q)
            if (max(Robots{j}.Beliefs{x}.Prob) == Robots{j}.Beliefs{x}.Prob(length(T.props)+1))%if more regions have the same probability including the empty observation (free space), consider the free space
                Region = length(T.props)+1;
            else
                [~,Region]= max(Robots{j}.Beliefs{x}.Prob);
            end
            Robots{j}.Beliefs{x}.Region = Region;
        end
    end
    %     plot([0,3.2,3.2,0,0],[0.8,0.8,2,2,0.8],'k','LineWidth',2)
end
