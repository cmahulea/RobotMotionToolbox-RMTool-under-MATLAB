function Robots = drawRobots(Robots)

for i=1:length(Robots)
%    txt = sprintf('r_%d',i);
%    Robots{i}.plotText = text(Robots{i}.x-1.1,Robots{i}.y,txt);
    Robots{i}.filledPose = plot(Robots{i}.x,Robots{i}.y,'ko','MarkerSize',4,'MarkerFaceColor','b');
end
