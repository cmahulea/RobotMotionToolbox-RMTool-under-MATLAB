function Robots = getObservations(Robots,T,Map)

N_r = length(Robots);
for i=1:N_r
    for x=1:length(T.Q)
        d = norm(Robots{i}.pos-T.mid{x}); %Distance to region
%         PYX = PYk_Xk_robotarium(d, Map{x}.Region, length(T.props)+1); %Sensor probabilities
        PYX = PYk_Xk_paware(d, Map{x}.Region, length(T.props)+1); %Sensor probabilities
        [~,Obs]= max(rand < cumsum(PYX)); %Observation
        Robots{i}.Observation{x}.Obs = Obs;
%         Likelyhood = PYk_Xk_robotarium(d, Obs, length(T.props)+1); %Likelyhood
        Likelyhood = PYk_Xk_paware(d, Obs, length(T.props)+1);
        Robots{i}.Observation{x}.Likelyhood = Likelyhood;
    end
end