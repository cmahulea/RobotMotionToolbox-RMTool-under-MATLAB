
% initial figures (empty environment)
init_fig = openfig('InitTrajBoolSpec.fig')';

% initial trajectories given by the result from CM & MK 2020 (Bool spec)
for r=1:length(rob_traj)    %plot trajectories of robots
    plot(rob_traj{r}(1,1),rob_traj{r}(2,1),data.rob_plot.line_color{r},...
        'Marker',data.rob_plot.marker{r},'LineWidth',data.rob_plot.line_width{r});
    plot(rob_traj{r}(1,:),rob_traj{r}(2,:),data.rob_plot.line_color{r},...
        'LineWidth',data.rob_plot.line_width{r});
    plot(rob_traj{r}(1,end),rob_traj{r}(2,end),data.rob_plot.line_color{r},...
        'Marker',data.rob_plot.marker{r},'LineWidth',data.rob_plot.line_width{r},'Color','r');
end

for r=1:length(rob_traj)    %plot trajectories of robots
    for tt = 1 : length(Sync_m)
        plot(rob_traj{r}(1,Sync_m{tt}),rob_traj{r}(2,Sync_m{tt}),data.rob_plot.line_color{r},...
            'Marker',data.rob_plot.marker{r},'LineWidth',data.rob_plot.line_width{r},'Color','b');
    end
end

for r=1:length(rob_traj)    %plot trajectories of robots
    for tt = 1 : length(Sync_m2)
        plot(rob_traj{r}(1,Sync_m2{tt}),rob_traj{r}(2,Sync_m2{tt}),data.rob_plot.line_color{r},...
            'Marker',data.rob_plot.marker{r},'LineWidth',data.rob_plot.line_width{r},'Color','b');
    end
end

%%dynamical release of resources

% find the orer of the robots
all_cells = zeros(1,size(Pre,1));
idx_end_val = zeros(1,length(rob_traj));
for r = 1:length(rob_traj)
    end_value = Run_cells(r,end);
    idx_end_val(1,r) = find(Run_cells(r,:) == end_value, 1,'first');
end

[idx_end_val, nr_rob] = sort(idx_end_val(1,:));
idx_end_val = [idx_end_val; nr_rob];

new_run_cells = ones(size(Run_cells));
new_run_cells(1,:) = Run_cells(idx_end_val(2,1),:);
pos_rob = cell(size(rob_traj));
pos_rob{1} = rob_traj{idx_end_val(2,1)};
size_traj = size(pos_rob{1});

for rr = 2:length(rob_traj) 
    temp_idx = [];
    temp_idx2 = [];
    prev_rob = idx_end_val(2,rr-1);
    current_rob = idx_end_val(2,rr);
    common = intersect(Run_cells(prev_rob,:), Run_cells(current_rob,:));
    pos_rob{rr} = rob_traj{current_rob}(:,end).*ones(size_traj);
    new_run_cells(rr,:) = Run_cells(current_rob,end).*new_run_cells(rr,:);
    for ii = 1:length(common)
        temp_idx = [temp_idx find(new_run_cells(rr-1,:) == common(ii),1,'last')];
        temp_idx2 = [temp_idx2 find(Run_cells(current_rob,:) == common(ii),1,'first')];

    end
    temp_idx_first = min(temp_idx);
    temp_length = length(temp_idx);
    temp_idx_first2 = min(temp_idx2);

    new_run_cells(rr,1:temp_idx_first) = Run_cells(current_rob,1:temp_idx_first);
    pos_rob{rr}(:,1:temp_idx_first) = rob_traj{current_rob}(:,1:temp_idx_first);

    smt = [temp_idx_first:temp_idx_first + temp_length-1];
    new_run_cells(rr,smt) = new_run_cells(rr,temp_idx_first);
    pos_rob{rr}(:,smt) = repmat(pos_rob{rr}(:,temp_idx_first),1,length(smt));

    idx_int_cells = find(Run_cells(current_rob,:) == new_run_cells(rr,temp_idx_first),1,'last');
    int_cells = unique(Run_cells(current_rob,idx_int_cells:end));
    dif2 = idx_end_val(1,rr) - temp_idx_first2;
    new_run_cells(rr,smt(end)+1:smt(end)+1 + dif2) = Run_cells(current_rob,temp_idx_first2:idx_end_val(1,rr));
    pos_rob{rr}(:,smt(end)+1:smt(end)+1 + dif2) = rob_traj{current_rob}(:,temp_idx_first2:idx_end_val(1,rr));

end


% initial figures (empty environment)
init_fig = openfig('InitTrajBoolSpec.fig')';
pause
data.rob_plot.line_color = {'g','c','m','k','y','r','b','g','c','m','k','y','r','b','g','c'};
% concurent movement of the robots
for uu = 1:size(pos_rob{1},2)-1
    for rr = 1:length(pos_rob)
        plot(pos_rob{rr}(1,uu:uu+1),pos_rob{rr}(2,uu:uu+1),data.rob_plot.line_color{rr},'LineWidth',data.rob_plot.line_width{rr});
    end
    pause(1);
end 





