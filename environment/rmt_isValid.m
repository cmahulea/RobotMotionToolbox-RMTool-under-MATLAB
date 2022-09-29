function bool = rmt_isValid(cellNumber, cellToMoveNumber, grid, visited, obstacleValue,Width)
    movIlegalDer = mod(cellNumber,Width) == 0 & cellToMoveNumber == cellNumber+1;
    movIlegalIzq = mod(cellNumber,Width) == 1 & cellToMoveNumber == cellNumber-1;
    movIlegalArriba = cellToMoveNumber > length(grid);
    movIlegalAbajo = cellToMoveNumber < 1;
    if (~movIlegalDer & ~movIlegalIzq & ~movIlegalArriba & ~movIlegalAbajo)
        %El movimiento se encuentra dentro de los limites
        movIlegalVisitado = visited(cellToMoveNumber);
        movIlegalObstaculo = grid(cellToMoveNumber) == obstacleValue;
        bool = ~movIlegalVisitado & ~movIlegalObstaculo;
    else
        %El movimiento se encuentra fuera de los limites
        bool = false;
    end
end