formula = regexprep(formula,'[pPoOY]','y'); %if user mistakenly uses for props letters p, P, o, O, or Y -> replace with lower-case "y"
numbers = regexp(formula, '\d*', 'Match'); %extract the numbers
reg_formula = unique(cellfun(@str2num, numbers));

all_reg = 1:Nobstacles; 
useless_reg = find(ismember(all_reg, reg_formula) == 0); % save the regions which are not used in LTL formula
aux_OBS_set = [];
k = 1;
for i = 1:size(OBS_set,2)
    if(isempty(find(ismember(useless_reg,OBS_set(i,:)),1)))
        aux_OBS_set = [aux_OBS_set; OBS_set(i,:)];
        k = k + 1;
    end
end

OBS_set = [aux_OBS_set; OBS_set(end,:)];