function Robots = ObservationCharacteristicMatrices(Robots,T)

%Obtain the matrices containing the characteristic vectors of the
%observations of all robots

for i = 1 : length(Robots)
    for j = 1 : length(T.Q)
        for k = 1 : length(T.props)
            if (Robots{i}.Beliefs{j}.Region == k)
                Robots{i}.V(k,j) = 1;
            else
                Robots{i}.V(k,j) = 0;
            end
        end
    end
end
