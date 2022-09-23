function trajectories = extract_trajectories(Run_cells, initialMarking)

% trajectories =  cell(length(Run_cells{1}),1);
% regions = cell2mat(Run_cells);

regions = Run_cells;
N_r = size(regions,1);
trajectories =  cell(N_r,1);

for r = 1:N_r
%     trajectories{r}.tray = regions(r,1);
    trajectories{r}.tray = initialMarking(r);
end

current = initialMarking;
for t = 2:size(regions,2)
    newreg = regions(find(ismember(regions(:,t),current)-1),t);
    for r = 1:N_r
        if ~ismember(current(r),regions(:,t))
            trajectories{r}.tray = [trajectories{r}.tray,newreg];
            current(r) = newreg;
        end
    end
end
