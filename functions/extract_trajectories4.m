function trajectories = extract_trajectories4(m0, sigma, Pre, Post, N_r)


nP = numel(m0);
[start,~]=find(Pre(:,sigma>0));
[goal,~]=find(Post(:,sigma>0));
ntrans = numel(start);
nfired = 0;
places = zeros(nP,ntrans+1);
places(:,1) = m0;
C = Post-Pre;

while sum(sigma)>1e-3
    mk = places(:,nfired+1);
%     Deltam = C*sigma;
    Pfireable = find(mk>0);
    TFireable = find(sum(Pre(Pfireable,:))>0);
    [~,IDsigmaFired] = max(sigma(TFireable));
    IDsigmaFired = TFireable(IDsigmaFired);
    IDPFired = find(Pre(:,IDsigmaFired));
    quantity = min(mk(IDPFired),sigma(IDsigmaFired));
    sigmaFired = zeros(length(sigma),1);
    sigmaFired(IDsigmaFired) = quantity;
    sigma(IDsigmaFired) = sigma(IDsigmaFired) - quantity;
    nfired = nfired+1;
    places(:,nfired+1) = places(:,nfired)+C*sigmaFired;
end
trajectories = [];

% N_r = size(places,1);
% trajectories =  cell(N_r,1);
% for r = 1:N_r
%     trajectories{r}.tray = places(r,1);
% end
% for t = 2:size(places,2)
%     newr = find(places(:,t)-places(:,t-1));
%     trajectories{newr}.tray = [trajectories{newr}.tray,places(newr,t)];
% end
