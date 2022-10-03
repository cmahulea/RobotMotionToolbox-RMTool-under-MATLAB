function boolSuccess = rmt_isBooleanFormulaTrue(formula, PN, RegionesRobots)
    formulaCmp = formula;    
    formula = erase(formula,'y'); %se quitan los caracteres 'y'
    formula = erase(formula,'!'); %se quitan los caracteres '!'
    tipoRegionRobots = unique(PN.T.obs(RegionesRobots));
    for i=1:PN.nlabels-2 %No contamos obstaculos ni celdas vacias
        indiceRegTxt = int2str(i);
        yInd = strcat('y', indiceRegTxt);
        if (ismember(i,tipoRegionRobots))
            formula = strrep(formula, indiceRegTxt, yInd);
        else
            formula = strrep(formula, indiceRegTxt, strcat('!', yInd));
        end
    end
    boolSuccess = true;
    [~,~,~,Atask1,btask1]=rmt_formula2constraints(char(formula), [], [], PN.nlabels);
    [~,~,~,Atask2,btask2]=rmt_formula2constraints(char(formulaCmp), [], [], PN.nlabels);
    for i=1:size(Atask1,1)
        if (sum(Atask2(i,:)~=0) > 1)
            %Es un OR, con hacer que al menos una variable sea true valdrá
            numVarOR = sum(Atask2(i,:)~=0);
            if (sum(Atask1(i,:)~=Atask2(i,:)) >= numVarOR)
                boolSuccess = false;
            end
        else
            if (sum(Atask1(i,:)~=Atask2(i,:)) ~= 0)
                boolSuccess = false;
            end
        end
    end
end
