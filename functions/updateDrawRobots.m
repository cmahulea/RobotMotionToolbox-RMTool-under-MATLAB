function Robots = updateDrawRobots(Robots)

for i=1:length(Robots)
    Robots{i}.plotText.Position = [Robots{i}.x-0.7,Robots{i}.y-0.7];
    Robots{i}.filledPose.XData = Robots{i}.x;
    Robots{i}.filledPose.YData = Robots{i}.y;
end