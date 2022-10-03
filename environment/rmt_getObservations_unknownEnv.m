function Robots = rmt_getObservations_unknownEnv(Robots,T,Map, prob)

N_r = length(Robots);
for i=1:N_r
    for x=1:length(T.Q)
        d = norm(Robots{i}.pos-T.mid{x}); %Distance to region
        PYX = rmt_PYk_Xk_paware_unknownEnv(d, Map{x}.Region, length(T.props)+1, prob); %Sensor probabilities
        [~,Obs]= max(rand < cumsum(PYX)); %Observation
        Robots{i}.Observation{x}.Obs = Obs;
        Likelyhood = rmt_PYk_Xk_paware_unknownEnv(d, Obs, length(T.props)+1, prob);
        Robots{i}.Observation{x}.Likelyhood = Likelyhood;
    end
end