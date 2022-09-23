function Robots = ProbabilisticCharacteristicMatrices(Robots,T)

%Obtain the matrices containing the characteristic vectors of the
%observations of all robots

for i = 1 : length(Robots)
    for j = 1 : length(T.Q)
        for k = 1 : length(T.props)
%            Robots{i}.W(k,j) = Robots{i}.Beliefs{j}.Prob(k);
           Robots{i}.W(k,j) = log(1-Robots{i}.Beliefs{j}.Prob(k));
           Robots{i}.W(k,j) = max(Robots{i}.W(k,j),-1000);
        end
    end
end
