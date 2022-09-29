function trajectories = extract_trajectories2(initialMarking, sigma, Pre, Post)

transitions = [];
sigma = round(sigma);
while sum(sigma)>0
    transitionsa = find(sigma);
    sigma(transitionsa) = sigma(transitionsa)-1;
    transitions = [transitions; transitionsa];
end
    
[start,~]=find(Pre(:,transitions));
[goal,~]=find(Post(:,transitions));
ntrans = numel(start);
fired = zeros(ntrans,1);
nfired = 0;
places = zeros(sum(initialMarking),ntrans+1);
places(:,1) = find(initialMarking);

while nfired < ntrans
    for i=1:ntrans
        robot = find(places(:,nfired+1) == start(i));
        if numel(robot)>1
            robot = robot(1);
        end
        if ~isempty(robot) && ~fired(i)
            fired(i) = 1;
            nfired = nfired+1;
            places(:,nfired+1) = places(:,nfired);
            places(robot,nfired+1) = goal(i);
        end
    end
end

N_r = size(places,1);
trajectories =  cell(N_r,1);
for r = 1:N_r
    trajectories{r}.tray = places(r,1);
end
for t = 2:size(places,2)
    newr = find(places(:,t)-places(:,t-1));
    trajectories{newr}.tray = [trajectories{newr}.tray,places(newr,t)];
end
