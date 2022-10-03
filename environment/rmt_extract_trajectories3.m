function trajectories = rmt_extract_trajectories3(initialPlaces, sigma, Pre, Post)

transitions = [];
sigma = round(sigma);
while sum(sigma)>0
    transitionsa = find(sigma);
    sigma(transitionsa) = sigma(transitionsa)-1;
    transitions = [transitions; transitionsa];
end

N_r = numel(initialPlaces);
[start,~]=find(Pre(:,transitions));
[goal,~]=find(Post(:,transitions));
ntrans = numel(start);
fired = zeros(ntrans,1);
nfired = 0;
places = zeros(N_r,ntrans+1);
places(:,1) = initialPlaces;

while nfired < ntrans
    for r = 1:N_r
        [tra,~]=find(start==places(r,nfired+1));
        if ~isempty(tra)
            nfired = nfired+1;
            places(:,nfired+1) = places(:,nfired);
            places(r,nfired+1) = goal(tra(1));
            goal(tra(1)) = [];
            start(tra(1)) = [];
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
