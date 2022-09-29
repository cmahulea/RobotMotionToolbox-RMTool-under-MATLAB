function T = create_partition(regions,bounds)

[C,adj,OBS_set,obs,mid_X,mid_Y,mid]=triangular_decomposition_regions(regions,bounds);

T.Q=1:length(C);
T.Vert=C;
T.adj=adj;
T.OBS_set=OBS_set;
T.obs=obs;
T.mid_X=mid_X;
T.mid_Y=mid_Y;
T.mid=mid;
%find cells corresponding to each defined region (proposition): 
T.props=cell(1,length(regions));  %props{i} will be row vector with indices of cells included in proposition(region) i
for i=1:length(regions)
    ind_obs=find(sum(T.OBS_set==i , 2));  %indices of observables containing prop. i
    T.props{i}=find(ismember(T.obs,ind_obs)); %searched cells
end
