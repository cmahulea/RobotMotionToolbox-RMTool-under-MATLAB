function rmt_create_unknown_environment()
    %Comportamiento implementado
    %Se crea un environment de forma aleatoria, pero no se resuelve,
    % de resolverlo se encarga el botón de pathPlanning
    
    data = get(gcf,'UserData');
    
    %% Introduccion de valores por parte del usuario
    %Se pide al usuario que introduzca los valores de entrada
    prompt = {'Number of robots', 'Number of region types'};
    dlg_title = 'Robot Motion Toolbox';
    num_lines = 1;
    defaultans = {'3', '4'};
    input_user = inputdlg(prompt,dlg_title,num_lines,defaultans);
    if isempty(input_user)
        return;
    end
    numRobots = char(input_user(1)); % Reading of robot's numbers from input interface
    numTiposRegiones = char(input_user(2)); % Reading of number of region types from input interface
    if (isempty(numRobots) || isempty(numTiposRegiones))
        return;
    end
    try
        numRobots = str2double(numRobots);
        numTiposRegiones = str2double(numTiposRegiones);
    catch
        uiwait(errordlg(sprintf('\nNumber of robots, radius and number of region types should be a natural number!'),'Robot Motion Toolbox','modal'));
        error('Number of robots, radius and number of region types should be a natural number!');
    end
    
    numRegionsPerType = zeros(1,numTiposRegiones + 1);
    
    %Ya se han introducido todos los valores
    
    %% Se limitan los ejes
    %Hay que cambiar los numeros que aparecen en los ejes por la
    %variable width (se utiliza el case "environment_limits")
    try
        temp(1) = 0;
        temp(2) = data.frame_limits(2);
        temp(3) = 0;
        temp(4) = data.frame_limits(4);
    catch
        uiwait(errordlg('Error introducing data!','Robot Motion Toolbox','modal'));
        return;
    end
    if ((temp(1) >= temp(2)) || (temp(3)>= temp(4)) || (temp(1)~=round(temp(1))) ...
            || (temp(2)~=round(temp(2))) || (temp(3)~=round(temp(3))) || (temp(4)~=round(temp(4))))
        uiwait(errordlg('Error introducing data!','Robot Motion Toolbox','modal'));
        return;
    end
    data.frame_limits = temp;
    cla(data.handle_env)
    set(data.handle_env,'xlim',[temp(1) temp(2)],'ylim',[temp(3) temp(4)],'XGrid','on','YGrid','on');
    set(gcf,'UserData',data);
    %Fin del case para limitar los ejes
    
    %% Inicio
    %Se obtiene el data y se añade la ruta de las funciones
    data = get(gcf,'UserData');

    %% General constants
    N_r = numRobots; % Number of robots (necesita ser igual o superior a los valores true de la funcion booleana, ej y1 & y2 & !y3, 2 o más robots)
    data.EstimationType = 3; %Estimation type (independent, centralized, consensus)
    data.DrawEstimations = 1; %Dibuja las modificaciones en los mapas

    Planner_Type.Deterministic = 0;
    Planner_Type.tau = 0.75;
    Planner_Type.Pcost = 1;
    Planner_Type.ObsC = 1;
    Planner_Type.tau_obs = 0.25;
    Planner_Type.Obscost = 1;
    Planner_Type.b = 1;

    data.RandomMoves = 0;
    data.totalDistance = 0;
    
    Width = data.frame_limits(2);
    Height = data.frame_limits(4);
    % Generate transitions for a square grid
    PN.T = rmt_generateTransitionPN(Height,Width);
    C = Width*Height;
    nlabels = numTiposRegiones + 2; %Types of regions (+2 debido a los obstaculos y las celdas vacias)
    
    %Colores de las regiones
    RegionColors = rand(nlabels,3); % Los colores de las regiones se inicializan aleatoriamente (luego se cambian las regiones 1 a 4, obstaculos y celdas vacias por unos colores concretos)
    RegionColors(1,:) = [1 1 0];
    RegionColors(2,:) = [0 0.5 0.75];
    RegionColors(3,:) = [1 0 0.5];
    RegionColors(4,:) = [0 0.5 0];
    RegionColors(nlabels-1,:) = [0 0 0]; % Pintar obstaculos de color negro
    RegionColors(nlabels,:) = [1 1 1];   % Pintar celdas vacias de color blanco
    
    RegionColors = RegionColors(1:nlabels,:);
    
    %Colores de los robots
    RobotColors = rand(N_r,3); % Los colores de los robots se inicializan aleatoriamente (luego se cambian los de los robots 1 a 5 por unos colores concretos)
    RobotColors(1,:) = [0.8,0,0];
    RobotColors(2,:) = [0,1,0];
    RobotColors(3,:) = [0,0,1];
    RobotColors(4,:) = [1,0,1];
    RobotColors(5,:) = [1,0.5,0];
    


    %% Petri Net generation
    set(gcf,'UserData',data);
    % Primero pregunta si se quiere crear el mapa de forma aleatoria o manual
    % Si se hace de forma aleatoria se crean las localizaciones de
    % las regiones y robots de forma aleatoria
    % Si se hace de forma manual se seleccionan las localizaciones
    % de las regiones y robots con clicks
    
    data = get(gcf,'UserData');
    % Menu to create the Environment
    % 1 Mode Manual
    % 2 Mode Random
    choiceMenuEnv = questdlg('How do you want to generate the environment?', ...
        'Robot Motion Toolbox', ...
        'Manual','Random','Random');
    if ((strcmpi(choiceMenuEnv,'Manual')))
        %Se crea el entorno de manera manual, con clicks
        data.randomEnv = 0;
    else
        %Se crea el entorno de forma aleatoria
        data.randomEnv = 1;
    end
    set(gcf,'UserData',data);


    data = get(gcf,'UserData');
    
    %PN.T.obs vector de C componenentes con los valores de cada celda (6 vacia, 5 obstaculo, 4 .. 1 regiones)
    PN.T.obs = nlabels*ones(1,C); % inicializa el vector con todo 6s (el num de regiones), el 6 indica celda blanca/vacia
    celdasDisponibles = (1:C);
    if (data.randomEnv == 1)
        %Se crea el entorno de forma aleatoria
        
        %Pedir por cada tipo de region cuantas celdas de ese tipo se quieren
        %numTiposRegiones es el numero de tipos de regiones posibles como meta,
        %es decir, no cuentan ni los obstaculos ni las celdas vacias
        %ej: Para la region 1 quiero 3, para la region 2 quiero 4, ...
        for i=1:numTiposRegiones+1 %+1 debido a las celdas obstaculo
            if i == numTiposRegiones+1
                txt = sprintf('Number of regions of type %d (obstacles):', i);
                prompt = {txt};
            else
                txt = sprintf('Number of regions of type %d:', i);
                prompt = {txt};
            end
            dlg_title = 'Robot Motion Toolbox';
            num_lines = 1;
            defaultans = {'2'};
            input_user = inputdlg(prompt,dlg_title,num_lines,defaultans);
            if isempty(input_user)
                return;
            end
            numRegions = char(input_user(1)); % Reading of number of type i regions
            if (isempty(numRegions))
                return;
            end
            try
                numRegions = str2double(numRegions);
            catch
                uiwait(errordlg(sprintf('\nNumber of regions should be a natural number!'),'Robot Motion Toolbox','modal'));
                error('Number of regions should be a natural number!');
            end
            numRegionsPerType(i) = numRegions;
        end
        
        numObstaculos = numRegionsPerType(numTiposRegiones+1);
        if (numObstaculos ~= 0)
            Sobstacles = celdasDisponibles(randperm(numel(celdasDisponibles),numObstaculos)); % guarda donde se ponen los obstaculos (celdas negras)
        else
            Sobstacles = [];
        end
        Socupadas = Sobstacles;
        celdasDisponibles = setdiff(celdasDisponibles, Socupadas);
        for k=1:nlabels-2
            numRegionesK = numRegionsPerType(k);
            Sk = [];
            if numRegionesK > 0
                Sk = celdasDisponibles(randperm(numel(celdasDisponibles),numRegionesK)); % Sk guarda donde colocar las regiones (celdas de colores ni blanco ni negro)
            end
            while ~isempty(intersect(Sk,Socupadas))
                Sk = celdasDisponibles(randperm(numel(celdasDisponibles),numRegionesK));
            end
            Socupadas = [Socupadas, Sk];
            celdasDisponibles = setdiff(celdasDisponibles, Socupadas);
            PN.T.obs(Sk) = k; % guarda en las posiciones Sk el valor de la region, k
            PN.T.props{1,k} = Sk; % PN.T.props guarda las posiciones para cada region
            %ej PN.T.props[1] tiene los n valores de celdas donde esta la region 1, por ejemplo [41, 92]
        end
        PN.T.obs(Sobstacles) = nlabels-1;
        PN.T.props{1,nlabels-1} = Sobstacles;

        R0 = ceil(rand(1,N_r)*C);
        while ~isempty(intersect(R0,Sobstacles))
            R0 = ceil(rand(1,N_r)*C);
        end
        PN.T.R0 = R0; % celdas iniciales de los N_r robots, ej [23, 64, 27, 82]
    else
        % Se crea el entorno de forma manual, con clicks
        % Código extraido de la función rmt_define_regions_grid
        env_bounds = data.frame_limits;
        obs_size = 1;
        reg_no = nlabels;
        robot_no = N_r;

        for i = env_bounds(1): obs_size : env_bounds(2)
            plot([i i],[env_bounds(3) env_bounds(4)],':r');
        end

        for i = env_bounds(3): obs_size : env_bounds(4)
            plot([env_bounds(1) env_bounds(2)],[i i],':r');
        end

        uiwait(msgbox(sprintf('\nUse left or right click to select a region'),...
            'Robot Motion Toolbox','modal'));

        try
            axes(data.handle_env);
        catch
            figure();
        end
        axis(env_bounds);
        hold on
        grid on
                
        for i=1:numTiposRegiones+1
            %Pido el número de regiones de tipo i
            if i == numTiposRegiones+1
                txt = sprintf('Number of regions of type %d (obstacles):', i);
                prompt = {txt};
            else
                txt = sprintf('Number of regions of type %d:', i);
                prompt = {txt};
            end
            dlg_title = 'Robot Motion Toolbox';
            num_lines = 1;
            defaultans = {'2'};
            input_user = inputdlg(prompt,dlg_title,num_lines,defaultans);
            if isempty(input_user)
                return;
            end
            numRegions = char(input_user(1)); % Reading of number of type i regions
            if (isempty(numRegions))
                return;
            end
            try
                numRegions = str2double(numRegions);
            catch
                uiwait(errordlg(sprintf('\nNumber of regions should be a natural number!'),'Robot Motion Toolbox','modal'));
                error('Number of regions should be a natural number!');
            end
            numRegionsPerType(i) = numRegions;
            
            for j=1:numRegionsPerType(i)
                added = 0;
                while (added == 0)
                    [x,y,~]=ginput(1);
                    if (x >= env_bounds(1) && x <= env_bounds(2) && y >= env_bounds(3) && y <= env_bounds(4)) %inside bounds
                        objects{i}=obs_size*[floor(x/obs_size) floor(x/obs_size)+1 floor(x/obs_size)+1 floor(x/obs_size);
                            floor(y/obs_size) floor(y/obs_size) floor(y/obs_size)+1 floor(y/obs_size)+1];
                        added = 1;
                        for k = 1 : i-1
                            if (norm(mean(objects{i},2)-mean(objects{k},2))<10^5*eps) %region already defined
                                added = 0;
                                uiwait(msgbox(sprintf('\nRegions of interest should be disjoint'),...
                                    'Robot Motion Toolbox','modal'));
                                break;
                            end
                        end
                    end
                end
                %drawing the regions
                pause(0.3)
                fill(objects{i}(1,:),objects{i}(2,:),'b-','FaceAlpha',0.5); %or functia patch (similara cu fill)
                coord = mean(objects{i},2);
                coordX = coord(1);
                coordY = coord(2);
                x = ceil(coordX);
                y = floor(coordY);
                numCelda = y * Width + x;
                PN.T.obs(numCelda) = i;
                Map{numCelda}.Region = PN.T.obs(numCelda);
                %Dibuja el color de la region (obtenido de draw_mapGT)
                Map{numCelda}.filledRegion = fill(PN.T.Vert{numCelda}(1,:),PN.T.Vert{numCelda}(2,:),RegionColors(Map{numCelda}.Region,:));
            end
        end

        for i = 1:reg_no-1
            PN.T.props{i} = find(PN.T.obs == i);
        end

        %Se pasa a seleccionar las posiciones iniciales de los robots
        uiwait(msgbox(sprintf('\nChoose the initial points of the robots with right click.\n'),'Robot Motion Toolbox','modal'));
        initial_point={};
        
        for k = 1:robot_no
            point_ok = 0;
            while(point_ok == 0)
                but=1;
                while but==1
                    [x,y,but]=ginput(1);
                end
                point_ok = 1;
                in = obs_size*[floor(x/obs_size) floor(x/obs_size)+1 floor(x/obs_size)+1 floor(x/obs_size);
                    floor(y/obs_size) floor(y/obs_size) floor(y/obs_size)+1 floor(y/obs_size)+1];
                for ll = 1 : length(objects)
                    if (norm(mean(objects{ll},2)-mean(in,2))<10^5*eps) %point inside a region of interest
                        uiwait(msgbox(sprintf('\nInitial point of the robots cannot be in a region of interest'),...
                            'Robot Motion Toolbox','modal'));
                        point_ok = 0;
                        break;
                    end
                end
                for ll = 1 : length(initial_point)
                    if (norm(initial_point{ll} - mean(in,2)) <= 10^5*eps)
                        uiwait(msgbox(sprintf('\nMaximum one robot inside a region'),...
                            'Robot Motion Toolbox','modal'));
                        point_ok = 0;
                        break;
                    end
                end
            end
            initial_point{k} = mean(in,2);
            plot(initial_point{k}(1),initial_point{k}(2),'or','LineWidth',3);
            coordX = initial_point{k}(1);
            coordY = initial_point{k}(2);
            x = ceil(coordX);
            y = floor(coordY);
            numCelda = y * Width + x;
            PN.T.R0(k) = numCelda;
        end

    end



    %compute the Petri net model (Pre and Post matrices)
    nplaces=length (PN.T.Q); %number of places
    Pre = zeros(nplaces,sum(sum(full(PN.T.adj))));
    Post = zeros(nplaces,sum(sum(full(PN.T.adj))));
    ntrans=0;
    %construct matrices Pre and Post
    for i=1:nplaces-1
        for j=i+1:nplaces
            if PN.T.adj(i,j)==1
                ntrans=ntrans+2;
                Pre(i,ntrans-1)=1;
                Pre(j,ntrans)=1;
                Post(i,ntrans)=1;
                Post(j,ntrans-1)=1;
            end
        end
    end
    PN.Pre = Pre;
    PN.Post = Post;
    PN.C_incidence=Post-Pre;
    PN.nplaces = nplaces;
    PN.ntrans = ntrans;
    PN.nlabels = nlabels;

    %% Initialize the map structure
    Map = cell(length(PN.T.Q),1);
    for x=1:length(Map)
        Map{x}.Region = PN.T.obs(x); % inicializa mapa con los tipos de region, numeros del 1 al 6
    end

    %% Initialize robotic structure
    Robots = cell(N_r,1);
    for i=1:N_r
        Robots{i}.reg = PN.T.R0(i); %numero de celda en la que esta el robot, ej 27
        Robots{i}.pos = PN.T.mid{PN.T.R0(i)}; % posicion concreta del robot, ej [6.5, 2.5] = [7 - 0.5, 2 - 0.5]
        Robots{i}.Path = Robots{i}.reg;
        Robots{i}.Beliefs = cell(length(PN.T.Q),1); %inicializa mapa de lo que cree ver el robot con ningun valor, []
        for x=1:length(PN.T.Q)
            Robots{i}.Beliefs{x}.Prob = 1/(length(PN.T.props)+1)*ones(length(PN.T.props)+1,1);
        end
        Robots{i}.TeamEstimatedPoses = PN.T.R0; % celdas iniciales de todos los robots
        Robots{i}.PlannedTrajectories = cell(N_r,1); % inicializa trayectorias para todos los robots con ningun valor, []
        for j=1:N_r
            Robots{i}.PlannedTrajectories{j}.tray = [];
        end
        Robots{i}.W = zeros(length(PN.T.props),length(PN.T.Q));
    end

    %% Initial Plots
    % Draw the initial GT map
    Map = rmt_draw_MapGT(PN.T,Map,RegionColors);

    % Draw the map in handle_env
    cla(data.handle_env);
    hold(data.handle_env,'on')
    for x=1:Width
        for y = 1:Height
            xreg = [x-1,x-1,x,x,x-1];
            yreg = [y-1,y,y,y-1,y-1];
            indice = (y-1) * Width + x;
            %Fill rellena del color de la region una celda
            fill(data.handle_env, xreg,yreg,RegionColors(Map{indice}.Region,:));
        end
    end
    for x=0:Width
        %handle_env
        plot(data.handle_env, [x,x],[0,Width],'k-');
        plot(data.handle_env, [0,Width],[x,x],'k-');
    end

    %Draw the initial robot positions in handle_env
    %handle_env
    for i=1:N_r
        Robots{i}.plot_handle_env = plot(data.handle_env,PN.T.mid{Robots{i}.reg}(1),PN.T.mid{Robots{i}.reg}(2),'.', 'MarkerSize',30,'Color',RobotColors(i,:));
        Robots{i}.textRobotIndex = text(data.handle_env,Robots{i}.pos(1)+0.2, Robots{i}.pos(2)-0.2, int2str(i));
    end

    cla(data.handle_ori);
    cla(data.handle_vel);
    cla(data.handle_ang);
    hold(data.handle_ori,'on')
    hold(data.handle_vel,'on')
    hold(data.handle_ang,'on')
    set(data.handle_ori,'xlim',[0 Width],'ylim',[0 Height],'XGrid','on','YGrid','on');
    set(data.handle_vel,'xlim',[0 Width],'ylim',[0 Height],'XGrid','on','YGrid','on');
    set(data.handle_ang,'xlim',[0 Width],'ylim',[0 Height],'XGrid','on','YGrid','on');
    set(data.handle_text,'Visible','off')

    text(data.handle_ori, Width/3.5,Height+0.7,'Belief Map: R1');
    text(data.handle_vel, Width/3.5,Height+0.7,'Belief Map: R2');
    text(data.handle_ang, Width/3.5,Height+0.7,'Belief Map: R3');
    

    if data.DrawEstimations
        % Draw the initial belief maps
        for i=1:length(Robots)
            %Dibuja las regiones en el belief map del robot i
            for x=1:length(PN.T.Q)
                if (max(Robots{i}.Beliefs{x}.Prob) == Robots{i}.Beliefs{x}.Prob(length(PN.T.props)+1))%if more regions have the same probability including the empty observation (free space), consider the free space
                    Region = length(PN.T.props)+1;
                else
                    [~,Region]= max(Robots{i}.Beliefs{x}.Prob);
                end
                Robots{i}.Beliefs{x}.Region = Region;
                if (i == 1)
                    Robots{i}.plotBeliefs{x} = fill(data.handle_ori,PN.T.Vert{x}(1,:),PN.T.Vert{x}(2,:),RegionColors(Region(1),:));
                elseif (i == 2)
                    Robots{i}.plotBeliefs{x} = fill(data.handle_vel,PN.T.Vert{x}(1,:),PN.T.Vert{x}(2,:),RegionColors(Region(1),:));
                elseif (i == 3)
                    Robots{i}.plotBeliefs{x} = fill(data.handle_ang,PN.T.Vert{x}(1,:),PN.T.Vert{x}(2,:),RegionColors(Region(1),:));
                end
            end
            
            %Dibuja las posiciones estimadas de los robots en el belief map del robot i
            for j=1:length(Robots)
                Robots{i}.plotTrajectories{j} = [];
                Robots{i}.plotGoal{j} = [];
                if (i == 1)
                    Robots{i}.plotStart{j} = plot(data.handle_ori,PN.T.mid{Robots{i}.TeamEstimatedPoses(j)}(1),PN.T.mid{Robots{i}.TeamEstimatedPoses(j)}(2),'o', 'LineWidth',2,'MarkerSize',10,'Color',RobotColors(j,:));
                elseif (i == 2)
                    Robots{i}.plotStart{j} = plot(data.handle_vel,PN.T.mid{Robots{i}.TeamEstimatedPoses(j)}(1),PN.T.mid{Robots{i}.TeamEstimatedPoses(j)}(2),'o', 'LineWidth',2,'MarkerSize',10,'Color',RobotColors(j,:));
                elseif (i == 3)
                    Robots{i}.plotStart{j} = plot(data.handle_ang,PN.T.mid{Robots{i}.TeamEstimatedPoses(j)}(1),PN.T.mid{Robots{i}.TeamEstimatedPoses(j)}(2),'o', 'LineWidth',2,'MarkerSize',10,'Color',RobotColors(j,:));
                end
            end
            Robots{i}.plotStart{i}.MarkerFaceColor=RobotColors(i,:);
        end
    end
    
    %we clean the orientation figure
    cla(data.handle_ori);
    %we clean the velocities figure
    cla(data.handle_vel);
    %we clean the steering angle figure
    cla(data.handle_ang);
    %enable the text information box
    set(data.handle_text,'Visible','on','String','');
    
    data.windowView = 1;
    set(data.viewWindow.informationBox,'Checked','on');
    set(data.viewWindow.beliefMaps,'Checked','off');
    set(data.viewWindow.motionControl,'Checked','off');
    
    %Escribe sobre handle_text la información de las regiones (celdas)
    message = sprintf('REGIONS OF INTEREST:');
    for i=1:numTiposRegiones+1
        if (i == numTiposRegiones+1) %obstaculos
            temp = sprintf('- Output y_{%d} (black) is for O_{%d} = \\{',i,i);
        else
            temp = sprintf('- Output y_{%d} is for O_{%d} = \\{',i,i);
        end
        for j=1:numRegionsPerType(i)-1
            temp = sprintf('%sc_{%d}, ',temp,PN.T.props{i}(j));
        end
        if (numRegionsPerType(i) ~= 0)
            temp = sprintf('%sc_{%d}',temp,PN.T.props{i}(numRegionsPerType(i)));       
        end
        temp = sprintf('%s\\}',temp);
        message = sprintf('%s\n%s',message,temp);
    end

    set(data.handle_text,'String',message,'Position',[0.35    0.054    0.63    0.2477]);
    data.messageInfBox = message;
    
    %Pasar las variables a data para poder leerlas de forma global
    data.PN = PN;
    data.N_r = N_r;
    data.Robots = Robots;
    data.Map = Map;
    data.Planner_Type = Planner_Type;
    data.Width = Width;
    data.Height = Height;
    data.C = C;
    data.RobotColors = RobotColors;
    data.RegionColors = RegionColors;

    figure(1);
    set(gcf,'UserData',data);
    
    %El entorno se ha podido definir
    data = get(gcf,'UserData');
    data.unknownEnvCreated = 1;
    set(gcf,'UserData',data);

end