function [path, minDist] = rmt_bfsAlgorithm(grid, startCell, destinationCell, obstacleValue, Width)
%% Generación del mapa
%Pasa a 0 todas las regiones no obstáculo (podemos pasar por ellas)
grid(grid~=obstacleValue) = 0;
%La celda inicial y final contienen los valores obstacleValue + 1 y
%obstacleValue + 2, respectivamente
startValue = obstacleValue + 1;
destinationValue = obstacleValue + 2;
grid(startCell) = startValue;
grid(destinationCell) = destinationValue;

%% Comienza el algoritmo a estrella
cell = struct('cellNumber',startCell,'distance',0,'path',startCell);
cell.cellNumber = startCell;
cell.distance = 0;
cell.path = startCell;
queue = [cell];

visited = zeros(1,length(grid));
visited(startCell) = 1;
minDist = inf;
path = startCell;

w = 0;

while(~isempty(queue))
    w = w + 1;
    if (w > 400)
        w = 0;
    end
    %Se obtiene y elimina el último elemento de la lista
    cell = queue(1);
    queue(1) = [];

    %Comprobamos que la celda sea la celda destino
    if (ismember(cell.cellNumber, destinationCell))
        minDist = cell.distance;
        path = cell.path;
        break
    end  

    % se mueve arriba
    if (rmt_isValid(cell.cellNumber, cell.cellNumber + Width, grid, visited,obstacleValue,Width))
        cellAux = cell;
        cellAux.cellNumber = cellAux.cellNumber + Width;
        cellAux.distance = cellAux.distance + 1;
        cellAux.path = [cellAux.path cellAux.cellNumber];
        queue = [queue cellAux];
        visited(cellAux.cellNumber) = 1;
    end

    % se mueve abajo
    if (rmt_isValid(cell.cellNumber, cell.cellNumber - Width, grid, visited,obstacleValue,Width))
        cellAux = cell;
        cellAux.cellNumber = cellAux.cellNumber - Width;
        cellAux.distance = cellAux.distance + 1;
        cellAux.path = [cellAux.path cellAux.cellNumber];
        queue = [queue cellAux];
        visited(cellAux.cellNumber) = 1;
    end

    % se mueve a la izquierda
    if (rmt_isValid(cell.cellNumber, cell.cellNumber - 1, grid, visited,obstacleValue,Width))
        cellAux = cell;
        cellAux.cellNumber = cellAux.cellNumber - 1;
        cellAux.distance = cellAux.distance + 1;
        cellAux.path = [cellAux.path cellAux.cellNumber];
        queue = [queue cellAux];
        visited(cellAux.cellNumber) = 1;
    end

    % se mueve a la derecha
    if (rmt_isValid(cell.cellNumber, cell.cellNumber + 1, grid, visited,obstacleValue,Width))
        cellAux = cell;
        cellAux.cellNumber = cellAux.cellNumber + 1;
        cellAux.distance = cellAux.distance + 1;
        cellAux.path = [cellAux.path cellAux.cellNumber];
        queue = [queue cellAux];
        visited(cellAux.cellNumber) = 1;
    end
end


end




