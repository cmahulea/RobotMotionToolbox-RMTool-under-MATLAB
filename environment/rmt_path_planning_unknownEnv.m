function rmt_path_planning_unknownEnv()    
    % Se activa la figura con tag 'figpriRMT' (ventana principal de rmtool)
    % De esta forma se puede obtener el data correspondiente a
    % esta figura
    handle = rmt_find_fig('figpriRMT');
    figure(handle);
    data=get(gcf,'UserData');
    if (~data.unknownEnvCreated)
        uiwait(errordlg(sprintf('\nDefine an unknown environment!'),'Robot Motion Toolbox','modal'));
        error('Define an unknown environment!');
    end

    %% High level formula definition
    formula = get(findobj(gcf,'Tag','booleanformula'),'String');
    if (any(isstrprop(formula,'upper')))
        uiwait(errordlg(sprintf('\nUppercase detected in boolean formula.\nFormula only takes in account final states, not trayectories.\nPlease change it.'),'Robot Motion Toolbox','modal'));
        return;
    end

    %% Se pregunta por los parámetros
    prompt = {'Estimation type (1:independent, 2:centralized, 3:consensus)', 'Communication radius', 'Tau', 'Tau_obs', 'MAXITERS'};
    dlg_title = 'Robot Motion Toolbox';
    num_lines = 1;
    defaultans = {'3', '25', '0.75', '0.25', '100'};

    input_user = inputdlg(prompt,dlg_title,num_lines,defaultans);
    if isempty(input_user)
        return;
    end
    EstimationType = char(input_user(1)); % Reading of estimation type from input interface
    radio = char(input_user(2)); % Reading of comm radius from input interface
    tau = char(input_user(3)); % Reading of tau from input interface
    tau_obs = char(input_user(4)); % Reading of tau_obs type from input interface
    MAXITERS = char(input_user(5)); % Reading of MAXITERS type from input interface
    if (isempty(EstimationType) || isempty(radio) || isempty(tau) || isempty(tau_obs))
        return;
    end
    try
        EstimationType = str2double(EstimationType);
        radio = str2double(radio);
        tau = str2double(tau);
        tau_obs = str2double(tau_obs);
        MAXITERS = str2double(MAXITERS);
    catch
        uiwait(errordlg(sprintf('\nEstimation type should be a natural number between 1 and 3!\nRadius should be a non negative number!\nTau and tau_obs should be numbers between 0 and 1!\nMAXITERS should be a positive number!'),'Robot Motion Toolbox','modal'));
        error('Estimation type should be a natural numberbetween 1 and 3!');
    end
    if (EstimationType < 1 || EstimationType > 3 || radio < 0 || tau > 1 || tau < 0 || tau_obs > 1 || tau_obs < 0 || MAXITERS <= 0)
        uiwait(errordlg(sprintf('\nEstimation type should be a natural number between 1 and 3!\nRadius should be a non negative number!\nTau and tau_obs should be numbers between 0 and 1!\nMAXITERS should be a positive number!'),'Robot Motion Toolbox','modal'));
        error('Estimation type should be a natural numberbetween 1 and 3!\nRadius should be a non negative number!\nTau and tau_obs should be numbers between 0 and 1!\nMAXITERS should be a positive number!');
    end

    data.CommRadius = radio;
    data.Planner_Type.tau = tau;
    data.Planner_Type.tau_obs = tau_obs;


    %% Se dibuja el data.Mapa y los data.Robots en sus pos iniciales en handle_env (borra lo que habia anteriormente)
    % Draw the data.Map in handle_env
    cla(data.handle_env);
    hold(data.handle_env,'on')
    for x=1:data.Width
        for y = 1:data.Height
            xreg = [x-1,x-1,x,x,x-1];
            yreg = [y-1,y,y,y-1,y-1];
            indice = (y-1) * data.Width + x;
            %Fill rellena del color de la region una celda
            fill(data.handle_env, xreg,yreg,data.RegionColors(data.Map{indice}.Region,:));
        end
    end
    for x=0:data.Width
        %handle_env
        plot(data.handle_env, [x,x],[0,data.Width],'k-');
        plot(data.handle_env, [0,data.Width],[x,x],'k-');
    end
    %Draw the initial robot positions in handle_env
    %handle_env
    for i=1:data.N_r
        data.Robots{i}.plot_handle_env = plot(data.handle_env,data.PN.T.mid{data.Robots{i}.reg}(1),data.PN.T.mid{data.Robots{i}.reg}(2),'.', 'MarkerSize',30,'Color',data.RobotColors(i,:));
        data.Robots{i}.textRobotIndex = text(data.handle_env,data.Robots{i}.pos(1)+0.2, data.Robots{i}.pos(2)-0.2, int2str(i));
    end

    %% Se dibujan los 3 belief data.Maps en handle_ori, handle_vel y handle_ang

    %Se desactiva el texto
    set(data.handle_text,'Visible','off','String','');

    cla(data.handle_ori);
    cla(data.handle_vel);
    cla(data.handle_ang);

    set(data.viewWindow.informationBox,'Checked','off');
    set(data.viewWindow.beliefMaps,'Checked','on');
    set(data.viewWindow.motionControl,'Checked','off');


    for i=1:length(data.Robots)

        for x=1:length(data.PN.T.Q)
            if (max(data.Robots{i}.Beliefs{x}.Prob) == data.Robots{i}.Beliefs{x}.Prob(length(data.PN.T.props)+1))%if more regions have the same probability including the empty observation (free space), consider the free space
                Region = length(data.PN.T.props)+1;
            else
                [~,Region]= max(data.Robots{i}.Beliefs{x}.Prob);
            end
            data.Robots{i}.Beliefs{x}.Region = Region;
            if (i == 1)
                data.Robots{i}.plotBeliefs{x} = fill(data.handle_ori,data.PN.T.Vert{x}(1,:),data.PN.T.Vert{x}(2,:),data.RegionColors(Region(1),:));
            elseif (i == 2)
                data.Robots{i}.plotBeliefs{x} = fill(data.handle_vel,data.PN.T.Vert{x}(1,:),data.PN.T.Vert{x}(2,:),data.RegionColors(Region(1),:));
            elseif (i == 3)
                data.Robots{i}.plotBeliefs{x} = fill(data.handle_ang,data.PN.T.Vert{x}(1,:),data.PN.T.Vert{x}(2,:),data.RegionColors(Region(1),:));
            end
        end
        for j=1:length(data.Robots)
            data.Robots{i}.plotTrajectories{j} = [];
            data.Robots{i}.plotGoal{j} = [];
            if (i == 1)
                data.Robots{i}.plotStart{j} = plot(data.handle_ori,data.PN.T.mid{data.Robots{i}.TeamEstimatedPoses(j)}(1),data.PN.T.mid{data.Robots{i}.TeamEstimatedPoses(j)}(2),'o', 'LineWidth',2,'MarkerSize',10,'Color',data.RobotColors(j,:));
            elseif (i == 2)
                data.Robots{i}.plotStart{j} = plot(data.handle_vel,data.PN.T.mid{data.Robots{i}.TeamEstimatedPoses(j)}(1),data.PN.T.mid{data.Robots{i}.TeamEstimatedPoses(j)}(2),'o', 'LineWidth',2,'MarkerSize',10,'Color',data.RobotColors(j,:));
            elseif (i == 3)
                data.Robots{i}.plotStart{j} = plot(data.handle_ang,data.PN.T.mid{data.Robots{i}.TeamEstimatedPoses(j)}(1),data.PN.T.mid{data.Robots{i}.TeamEstimatedPoses(j)}(2),'o', 'LineWidth',2,'MarkerSize',10,'Color',data.RobotColors(j,:));
            end
        end
        data.Robots{i}.plotStart{i}.MarkerFaceColor=data.RobotColors(i,:);
    end

    %Se escribe el texto 'Belief data.Map:R{i}'
    text(data.handle_ori, data.Width/3.5,data.Height+0.7,'Belief Map: R1');
    if data.N_r > 1
        text(data.handle_vel, data.Width/3.5,data.Height+0.7,'Belief Map: R2');
    end
    if data.N_r > 2
        text(data.handle_ang, data.Width/3.5,data.Height+0.7,'Belief Map: R3');
    end


    %% Simulation of estimation rounds
    iterations = 0;
    stopped_data.Robots = zeros(1,data.N_r);
    numItStopped = 0;
    metaAlcanzada = false;
    % while ~(sum(stopped_data.Robots) == data.N_r) && (data.PN.T<data.numIterations)
    %while (numItStopped < 5 && iterations<data.numIterations)
    while (iterations<MAXITERS && ~metaAlcanzada)
        iterations= iterations+1;
        fprintf(1,'\nIteration: %d - ',iterations);
        data.RobotsRegItAnterior = [];
        for i=1:data.N_r
            data.RobotsRegItAnterior = [data.RobotsRegItAnterior, data.Robots{i}.reg];
        end
        
        %% Compute the communication graph based on the positions of the data.Robots
        G = rmt_computeCommGraph(data.Robots,data.CommRadius);
        W = rmt_WeightedMatrix(G,1);
        %% Update estimated positions for neighbor data.Robots
        for i=1:data.N_r
            data.Robots{i}.TeamEstimatedPoses(i) = data.Robots{i}.reg;
            for j=1:data.N_r
                data.Robots{i}.PlannedTrajectories{j}.tray = data.Robots{i}.TeamEstimatedPoses(j);
                if G(i,j)
                    data.Robots{i}.TeamEstimatedPoses(j) = data.Robots{j}.reg;
                end
            end
        end
        %% Each robot acquires a measurement of each cell of the data.Map
        data.Robots = rmt_getObservations_unknownEnv(data.Robots,data.PN.T,data.Map, 1);
        %% Estimation (BAYES RULE)
        data.Robots = rmt_updateBeliefs(data.Robots,data.PN.T,W,EstimationType);                    

        %% Update the belief data.Maps (Plots)
        if data.DrawEstimations
            data.Robots = rmt_draw_BeliefMapsUpdate(data.Robots,data.PN.T,data.RegionColors); % Cambia FaceColor: data.Robots{i}.plotBeliefs{x}.FaceColor
            data.Robots = rmt_draw_Trajectories(data.Robots,data.PN.T,data.RobotColors);
        end

        %% Path planning

        %Vector con los numeros de celdas de los obstaculos (ej : [3,17,52,73,90])
        posicionesObstaculos = find(data.PN.T.obs == data.PN.nlabels - 1);

        for i=1:data.N_r
            MILP_data = rmt_createMILP_unknownEnv(data.Robots,data.PN,formula,i,data.Planner_Type);
            options=optimoptions('intlinprog','Display','off');
            [xmin,fval,exitflag] = intlinprog(MILP_data.f,1:size(MILP_data.Aineq,2),...
                MILP_data.Aineq,MILP_data.bineq,...
                MILP_data.Aeq,MILP_data.beq,...
                MILP_data.lb,MILP_data.ub,[],options);
            if (exitflag == 1) %solution found
                sigma=xmin(data.PN.nplaces+1:data.PN.nplaces+data.PN.ntrans,1); %the firing vector of the solution
                trajectories = rmt_extract_trajectories3(data.Robots{i}.TeamEstimatedPoses, sigma, data.PN.Pre, data.PN.Post);

                % Evitar que se mueva a un obstaculo
                % Si su tray es ilegal o un obstaculo entonces
                % no moverse, quedarse en la celda actual
                for j=1:data.N_r
                    if length(trajectories{j}.tray) > 1
                        if (ismember(trajectories{j}.tray(2), posicionesObstaculos))
                            trajectories{j}.tray(2) = data.Robots{j}.reg;
                        end
                    else
                        if (ismember(trajectories{j}.tray(1), posicionesObstaculos))
                            trajectories{j}.tray(1) = data.Robots{j}.reg;
                        end
                    end
                end

                %Evitar que se mueva a una celda ilegal (no vecina)
                %adj_reg: celdas vecinas (contando la celda
                %actual del robot) sin tener en cuenta las
                %celdas obstaculo
                adj_reg = find(data.PN.T.adj(data.Robots{i}.reg,:)); %adyacent regions of robot i
                for j = 1:length(data.Robots)
                    adj_reg = setdiff(adj_reg,data.Robots{j}.reg); %remove the adyacent regions occupied by other data.Robots
                end
                numCeldaObstaculo = max(data.PN.T.obs) - 1;
                adj_reg = setdiff(adj_reg,find(data.PN.T.obs == numCeldaObstaculo)); %quitar vecinos que sean obstaculos
                adj_reg = [adj_reg, data.Robots{i}.reg]; %Se añade la celda en la que se encuentra el robot
                %A continuacion se evita que un robot pueda
                %saltar a una celda ilegal, es decir, que no
                %sea vecina o la actual donde esté el robot
                if length(trajectories{i}.tray) > 1
                    if (~ismember(trajectories{i}.tray(2), adj_reg))
                        trajectories{i}.tray(2) = data.Robots{i}.reg;
                    end
                else
                    if (~ismember(trajectories{i}.tray(1), adj_reg))
                        trajectories{i}.tray(1) = data.Robots{i}.reg;
                    end
                end

                data.Robots{i}.PlannedTrajectories = trajectories;

                if length(trajectories{i}.tray) > 1
                    fprintf(1,'\nRobot %d from %d moves to %d',i,data.Robots{i}.reg,trajectories{i}.tray(2));

                else
                    fprintf(1,'\nRobot %d from %d moves to %d',i,data.Robots{i}.reg,trajectories{i}.tray(1));

                end
            else %no solution found by solving the MILP
                switch exitflag
                    case 6
                        fprintf(1,'\nNon-optimal Solution available.');
                    case 5
                        fprintf(1,'\nSolution with numerical issues.');
                    case 1
                        fprintf(1,'\nFunction converged to a solution x.');
                    case 0
                        fprintf(1,'\nNumber of iterations exceeded options.MaxIter.');
                    case -1
                        fprintf(1,'\nAborted.');
                    case -2
                        fprintf(1,'\nNo feasible point was found.');
                    case -3
                        fprintf(1,'\nProblem is unbounded.');
                    case -4
                        fprintf(1,'\nNaN value was encountered during execution of the algorithm.');
                    case -5
                        fprintf(1,'\nBoth primal and dual problems are infeasible.');
                    case -7
                        fprintf(1,'\nSearch direction became too small. No further progress could be made.');
                    case -8
                        fprintf(1,'\nProblem is infeasible or unbounded.');
                    case -9
                        fprintf(1,'\nLimit reached.');
                end

                %El robot en lugar de moverse de forma
                %aleatoria irá a zonas del mapa que no estén exploradas

                
                %Se obtienen las celdas sin explorar más cercanas
                %al robot para saber a donde tiene que ir
                minDist2 = inf;
                celdasAExplorarMasCercanas = [];
                numCeldaObstaculo = max(data.PN.T.obs) - 1;
                for x=1:data.C
                    if (data.Robots{i}.Beliefs{x}.Region ~= numCeldaObstaculo)
                        if (max(data.Robots{i}.Beliefs{x}.Prob) < 2/max(data.PN.T.obs))
                            %Obtener distancia a celda no explorada
                            d = norm(data.Robots{i}.pos - data.PN.T.mid{x});
                            if (d == minDist2)
                                celdasAExplorarMasCercanas = [celdasAExplorarMasCercanas x];
                            end
                            if (d < minDist2)
                                minDist2 = d;
                                celdasAExplorarMasCercanas = [];
                                celdasAExplorarMasCercanas = [celdasAExplorarMasCercanas x];
                            end
                        end
                    end
                end

                %Se obtiene los valores de las regiones del
                %belief data.Map
                gridBelief = [];
                for b=1:data.C
                    gridBelief(b) = data.Robots{i}.Beliefs{b}.Region;
                end

                %adj_reg se usa para hallar la min distancia
                %bfs desde las posibles posiciones a avanzar
                [ruta, minDistObtenida] = rmt_bfsAlgorithm(gridBelief, data.Robots{i}.reg, celdasAExplorarMasCercanas, numCeldaObstaculo, data.Width);
                %ruta(1): posicion inicial, ruta(2): celda del primer movimiento
                if(length(ruta) == 1)
                    celdaAvanzar = ruta(1);
                else
                    celdaAvanzar = ruta(2);
                end

                %El robot ya no se mueve aleatoriamente
                % Ahora va hacia la zona del data.Mapa menos explorada
                fprintf(1,'\nRobot %d will move to a not explored zone',i);
                fprintf(1,'\nRobot %d from %d moves to %d',i,data.Robots{i}.reg,celdaAvanzar);



                data.Robots{i}.PlannedTrajectories{i}.tray = [data.Robots{i}.reg,celdaAvanzar];

            end
        end




        %% Move the data.Robots in handle_env (SIMULATION)
        for i = 1 : data.N_r 
            for j = 1 : data.N_r
                Tray=data.Robots{i}.PlannedTrajectories{j}.tray;
                if length(Tray) > 1
                    data.Robots{i}.TeamEstimatedPoses(j) = Tray(2);
                end
            end

            data.Robots{i}.reg = data.Robots{i}.TeamEstimatedPoses(i);
            data.Robots{i}.pos = data.PN.T.mid{data.Robots{i}.TeamEstimatedPoses(i)};
            data.Robots{i}.Path = [data.Robots{i}.Path,data.Robots{i}.reg];

        end

        for i=1:data.N_r
            data.Robots{i}.plot_handle_env.XData = data.PN.T.mid{data.Robots{i}.reg}(1);
            data.Robots{i}.plot_handle_env.YData = data.PN.T.mid{data.Robots{i}.reg}(2);
            delete(data.Robots{i}.textRobotIndex);
            data.Robots{i}.textRobotIndex = text(data.handle_env,data.Robots{i}.pos(1)+0.2, data.Robots{i}.pos(2)-0.2, int2str(i));
        end

        %% Update the belief data.Maps (Plots)
        if data.DrawEstimations
            data.Robots = rmt_draw_BeliefMapsUpdate(data.Robots,data.PN.T,data.RegionColors); % Cambia FaceColor: data.Robots{i}.plotBeliefs{x}.FaceColor
            data.Robots = rmt_draw_Trajectories(data.Robots,data.PN.T,data.RobotColors);
        end

        %% End condition
        data.RobotsRegItActual = [];
        for i=1:data.N_r
            data.RobotsRegItActual = [data.RobotsRegItActual, data.Robots{i}.reg];
        end
        if (data.RobotsRegItAnterior == data.RobotsRegItActual)
            numItStopped = numItStopped + 1;
        else
            numItStopped = 0;
        end

        %Vemos si la fórmula booleana se cumple
        metaAlcanzada = rmt_isBooleanFormulaTrue(formula, data.PN, data.RobotsRegItActual);

        pause(0.001);
    end

    %% Update the Real data.Map with the data.Robots (Plots)
    data.Robots = rmt_draw_FinalTrajectories(data.Robots,data.PN.T);

    for i=1:length(data.Robots)    
        Tray=data.Robots{i}.Path;
        if ~isempty(Tray)
            trayx = zeros(1,length(Tray));
            trayy = zeros(1,length(Tray));
            for k = 1:length(Tray)
                trayx(k) = data.PN.T.mid{Tray(k)}(1);
                trayy(k) = data.PN.T.mid{Tray(k)}(2);
            end
            colorTransparente = data.RobotColors(i,:);
            colorTransparente(4) = 0.4;
            plot(data.handle_env, trayx, trayy, '-', 'LineWidth',4, 'Color', colorTransparente,'MarkerFaceColor',data.RobotColors(i,:));
            plot(data.handle_env, trayx(1),trayy(1),'*', 'LineWidth',1,'MarkerSize',10,'Color',colorTransparente);
        end
    end
    
    fprintf(1,'\nPath planning ended.\n');
    
end