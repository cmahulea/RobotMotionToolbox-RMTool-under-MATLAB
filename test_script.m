C = data.T.Vert;

for i=1:length(C)
        text(centr(1),centr(2),sprintf('%s_{%d}','c',i),'HorizontalAlignment','center','Color','w','FontSize',10,'FontAngle','italic','FontName','TimesNewRoman');
        fill(C{i}(1,:),C{i}(2,:),'w','EdgeColor',[0.8 0.8 0.8],'FaceAlpha',0.5);

        if i == 4 || i == 10 
    fill(C{i}(1,:),C{i}(2,:),'g','EdgeColor',[0 1 0],'FaceAlpha',0.5);
    elseif i == 13 || i==25 || i==19 || i == 20 || i== 18 || i==27 || i == 16
      fill(C{i}(1,:),C{i}(2,:),'b','EdgeColor',[0 0 1],'FaceAlpha',0.5);
    elseif i == 21 || i == 17 || i == 16 || i == 27
    fill(C{i}(1,:),C{i}(2,:),'r','EdgeColor',[1 0 0],'FaceAlpha',0.5);
    else
    fill(C{i}(1,:),C{i}(2,:),'w','EdgeColor',[0.8 0.8 0.8],'FaceAlpha',0.5);

    end
end

for i=1:length(C)
 
    fill(C{i}(1,:),C{i}(2,:),'w','EdgeColor',[0.3 0.3 0.3],'FaceAlpha',0.5);
        if i == 4 || i == 10 
    fill(C{i}(1,:),C{i}(2,:),'g','EdgeColor',[0 1 0],'FaceAlpha',0);
    elseif i == 13 || i==25 || i==19 || i == 20 || i== 18 || i==27 || i == 16
      fill(C{i}(1,:),C{i}(2,:),'b','EdgeColor',[0 0 1],'FaceAlpha',0);
    elseif i == 21 || i == 17 || i == 16 || i == 27
    fill(C{i}(1,:),C{i}(2,:),'r','EdgeColor',[1 0 0],'FaceAlpha',0);
    else
    fill(C{i}(1,:),C{i}(2,:),'w','EdgeColor',[0 0 0],'FaceAlpha',0.5);
        end
end

for i=1:length(C)
    centr=mean(C{i},2)';
    text(centr(1),centr(2),sprintf('%s_{%d}','p',i),'HorizontalAlignment','center','Color','k','FontSize',10,'FontName','TimesNewRoman');
end

rob = data.initial;
for i = 1:length(rob)
    plot(rob{i}(1),rob{i}(2),'Color',[0 0 0],'LineWidth',2,'Marker','o');
end

%% trajectories
traj = data.trajectory;

for i = 1:length(traj)
    line(traj{i}(1,:)',traj{i}(2,:)','Color','k','Linewidth',2);
    plot(traj{i}(1,end),traj{i}(2,end),'Color',[1 0 0], 'LineWidth',1,'Marker','*')
end