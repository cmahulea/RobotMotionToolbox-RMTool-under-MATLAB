function Map = updateRegions(Map,RegionColors)

for x=1:length(Map)
    Map{x}.filledRegion.FaceColor = RegionColors(Map{x}.typeRegion,:);
end