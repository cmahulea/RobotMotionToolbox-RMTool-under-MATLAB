function Map = drawRegions(T,Map,RegionColors)

for x=1:length(Map)
    Map{x}.filledRegion = fill(T.Vert{x}(1,:),T.Vert{x}(2,:),RegionColors(Map{x}.typeRegion,:));
    Map{x}.RegionNumber = text(T.mid{x}(1),T.mid{x}(2),sprintf('p_{%d}',x),'HorizontalAlignment','center','Color','k','FontSize',10,'FontAngle','italic','FontName','TimesNewRoman');
end
