function Robots = rmt_PlanningEstimation(numRobots, radioAlcance, ancho, numRegiones)

close all
rng(1);

%% General constants
N = numRobots; % Number of robots
NumSubplots = ceil(sqrt(N)); %For plot purposes
CommRadius = radioAlcance;
Width = ancho; %Size of the map
NumTypeRegions = numRegiones; %Types of regions
numIterations = 100; %Number of estimation rounds
EstimationType = 3; %Estimation type (independent, centralized, consensus)

%% Initialize the map structure
Map = cell(Width,Width);
for x=1:Width
    for y=1:Width
        Map{x,y}.typeRegion = ceil(NumTypeRegions*rand);
    end
end
ProbXk1Xk = eye(NumTypeRegions); %Regions dont change type
% Se definen los colores de las regiones
% Tipo 1: rojo, 2: azul, 3: verde, 4 en adelante: aleatorio
RegionColors= [1,0,0;...
       0,0,1;...
       0,1,0];
RegionColors = [RegionColors;rand(NumTypeRegions-3,3)];
% Draw the map
figure(1);
hold on
for x=1:Width
    for y=1:Width
        xreg = [x-1,x-1,x,x,x-1];
        yreg = [y-1,y,y,y-1,y-1];
        % Colorea la region x y segun su tipo de region
        Map{x,y}.filledRegion = fill(xreg,yreg,RegionColors(Map{x,y}.typeRegion,:));
    end
end
for x=0:Width
    plot([x,x],[0,Width],'k-');
    plot([0,Width],[x,x],'k-');
end

%% Initialize robotic structure
Robots = cell(N,1);
for i=1:N
    % Se define la posicion del robot en x y
    Robots{i}.x = ceil(Width*rand);
    Robots{i}.y = ceil(Width*rand);
    Robots{i}.pos = [Robots{i}.x,Robots{i}.y];
    % Se inicializa el mapa que cree ver el robot (al principio esta vacio)
    % Beliefs guarda un mapa de dim width x width donde cada celda guarda
    % la probabilidad de que la celda sea de un tipo especifico
    % Ej: Prob = [0,1,0,0,0,0,0,0,0] el robot cree que la celda es de tipo 2
    Robots{i}.Beliefs = cell(Width,Width);
    for x=1:Width
        for y=1:Width
            % Al principio los robots tienen todas las probabilidades para
            % todas las celdas con el mismo valor, por lo que no hay ningun
            % valor que sea el mayor
            % Ej: Prob = [0.33, 0.33, 0.33] si solo huubiese 3 tipos de region
            Robots{i}.Beliefs{x,y}.Prob = 1/NumTypeRegions*ones(NumTypeRegions,1);
        end
    end
    % Observation = mapa donde cada celda tiene -> {Obs, Likelyhood}
    % Likelyhood = matriz con las probabilidades del tipo de region Ej: {0, 0.2, 0.8} si hay 3 tipos de region
    Robots{i}.Observation = cell(Width,Width);
end
RobotColors  = 0;
% Draw the robots in the general map
for i=1:N
    txt = sprintf('%d',i);
    Robots{i}.plotText = text(Robots{i}.x-0.7,Robots{i}.y-0.7,txt);
    Robots{i}.filledPose = plot(Robots{i}.x-0.5,Robots{i}.y-0.5,'ko','MarkerSize',10,'MarkerFaceColor','k');
end
% Draw the belief maps 
figure(2);
for i=1:N
    subplot(NumSubplots,NumSubplots,i);
    hold on
    for x=1:Width
        for y=1:Width
            xreg = [x-1,x-1,x,x,x-1];
            yreg = [y-1,y,y,y-1,y-1];
            [~,Region]= max(Robots{i}.Beliefs{x,y}.Prob);
            Robots{i}.Beliefs{x,y}.Region = Region;
            Robots{i}.Beliefs{x,y}.filledRegion= fill(xreg,yreg,RegionColors(Region(1),:));
        end
    end
    for x=0:Width
        plot([x,x],[0,Width],'k-');
        plot([0,Width],[x,x],'k-');
    end
end

%% Simulation of estimation rounds
for t=1:numIterations
    %% Compute the communication graph based on the positions of the robots
    G = eye(N);
    for i=1:N
        posi = Robots{i}.pos;
        for j=i+1:N
            posj = Robots{j}.pos;
            dist = norm(posi-posj);
            if dist <= CommRadius
                % La pareja de robots i j se encuentran cerca, pueden comunicarse
                % G(i, j) = 1 significa que el robot i se puede comunicar con el j
                % G es una matriz simetrica que sirve para saber que robots pueden comunicarse entre si
                G(i,j) = 1;
                G(j,i) = 1;
            end
        end
    end
    W = rmt_weighted_matrix(G,1);
    %% Each robot acquires a measurement of each cell of the map
    for i=1:N
        for x=1:Width
            for y=1:Width
                d = norm(Robots{i}.pos-[x,y]); %Distance to region
                PYX = rmt_PYk_Xk(d, Map{x,y}.typeRegion, NumTypeRegions); %Sensor probs
                [~,Obs]= max(rand < cumsum(PYX)); %Observation
                Likelyhood = rmt_PYk_Xk(d, Obs, NumTypeRegions); %Likelyhood
                Robots{i}.Observation{x,y}.Obs = Obs;
                Robots{i}.Observation{x,y}.Likelyhood = Likelyhood;
            end
        end
    end
    %% Estimation (BAYES RULE)
    switch EstimationType
        case 1 %Each robot acts independently
            for i=1:N
                for x=1:Width
                    for y=1:Width
                        Likelyhood = Robots{i}.Observation{x,y}.Likelyhood;
                        scale = Likelyhood'*Robots{i}.Beliefs{x,y}.Prob;
                        posterior = Robots{i}.Beliefs{x,y}.Prob.*Likelyhood/scale;
                        belief = zeros(NumTypeRegions,1);
                        for k=1:NumTypeRegions
                            belief(k) = ProbXk1Xk(k,:)*posterior;
                        end
                        belief = belief/sum(belief);
                        Robots{i}.Beliefs{x,y}.Prob = belief;
                    end
                end
            end
        case 2 %Centralized
            for x=1:Width
                for y=1:Width
                    Likelyhood = ones(NumTypeRegions,1);
                    for i=1:N
                        Likelyhood = Likelyhood.*Robots{i}.Observation{x,y}.Likelyhood;
                    end
                    for i=1:N
                        scale = Likelyhood'*Robots{i}.Beliefs{x,y}.Prob;
                        posterior = Robots{i}.Beliefs{x,y}.Prob.*Likelyhood/scale;
                        belief = zeros(NumTypeRegions,1);
                        for k=1:NumTypeRegions
                            belief(k) = ProbXk1Xk(k,:)*posterior;
                        end
                        belief = belief/sum(belief);
                        Robots{i}.Beliefs{x,y}.Prob = belief;
                    end
                end
            end
        case 3 %Consensus
            for x=1:Width
                for y=1:Width
                    pi0 = zeros(NumTypeRegions,N);
                    for i=1:N
                        pi0(:,i) = Robots{i}.Observation{x,y}.Likelyhood;
                    end
                    pik = pi0;
                    for it=1:100
                        piN = ones(NumTypeRegions,N);
                        for i=1:N
                            for j=1:N
                                if G(i,j)
                                    piN(:,i)=piN(:,i).*pik(:,j).^W(i,j);
                                end
                            end
                        end
                        pik = piN;
                    end
                    for i=1:N
                        pik(:,i) = (pik(:,i).^N)/sum(pik(:,i).^N);
                        Likelyhood = pik(:,i);
                        Robots{i}.Observation{x,y}.Likelyhood = pik(:,i);
                        scale = Likelyhood'*Robots{i}.Beliefs{x,y}.Prob;
                        posterior = Robots{i}.Beliefs{x,y}.Prob.*Likelyhood/scale;
                        belief = zeros(NumTypeRegions,1);
                        for k=1:NumTypeRegions
                            belief(k) = ProbXk1Xk(k,:)*posterior;
                        end
                        belief = belief/sum(belief);
                        % Se modifica Prob del robot, siendo esto un vector
                        % con las probabilidades de cada tipo de region,
                        % Ej: Prob para celda x y de robot i = {0.2, 0.7, 0.1}
                        Robots{i}.Beliefs{x,y}.Prob = belief;
                    end
                end
            end
    end
    %% Update the map with the robots (Plots)
    % Modifica el mapa real con la posicion de los robots como puntos en el
    % centro de la celda en la que se encuentran
    figure(1);
    for i=1:N
        % IMPORTANTE: Estas tres lineas son las que modifican la posicion
        % del robot en el mapa
        Robots{i}.plotText.Position = [Robots{i}.x-0.7,Robots{i}.y-0.7];
        Robots{i}.filledPose.XData = Robots{i}.x-0.5;
        Robots{i}.filledPose.YData = Robots{i}.y-0.5;
    end
    %% Update the belief maps (Plots)
    % Modifica los mapas que creen ver los robots
    figure(2);
    for i=1:N
        subplot(NumSubplots,NumSubplots,i);
        for x=1:Width
            for y=1:Width
                [~,Region]= max(Robots{i}.Beliefs{x,y}.Prob);
                if Robots{i}.Beliefs{x,y}.Region ~= Region
                    % IMPORTANTE: Estas dos lineas son las que modifican el
                    % color de las regiones en los mapas que creen ver los
                    % robots
                    Robots{i}.Beliefs{x,y}.filledRegion.FaceColor = RegionColors(Region(1),:);
                    Robots{i}.Beliefs{x,y}.Region = Region;
                end
            end
        end
    end
    %% Move the robots to and adyacent random cell in the map
    for i=1:N
        movex = round(2*rand-1);
        if (Robots{i}.x+movex)>0 && (Robots{i}.x+movex)<=Width
            Robots{i}.x = Robots{i}.x + movex;
        end
        movey = round(2*rand-1);
        if (Robots{i}.y+movey)>0 && (Robots{i}.y+movey)<=Width
            Robots{i}.y = Robots{i}.y + movey;
        end
        Robots{i}.pos = [Robots{i}.x,Robots{i}.y];
    end
    pause(0.05);
end