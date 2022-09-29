function MILP_data = createMILP(Robots,PN,formula,i,planner)
%Construct the MILP

nplaces=PN.nplaces;
ntrans = PN.ntrans;
nlabels = PN.nlabels;
N_r = length(Robots);
N = 1000;

%initialize m0 (matriz booleana que indica para cada robot las celdas estimadas donde se encuentran los robots)
MILP_data.m0=zeros(nplaces,1);
for j=1:N_r
    MILP_data.m0(Robots{i}.TeamEstimatedPoses(j))=MILP_data.m0(Robots{i}.TeamEstimatedPoses(j))+1;
end

% Upper (ub) and lower bounds (lb)
MILP_data.lb = zeros(nplaces+ntrans+nlabels,1);
MILP_data.ub = [ones(nplaces,1);inf(ntrans+nlabels,1)];

% Compute matrix W
W = zeros(nlabels,nplaces);
if planner.Deterministic
    for j = 1 : nplaces
        for k = 1 : nlabels
            if (Robots{i}.Beliefs{j}.Region == k)
                W(k,j) = 1;
            end
        end
    end
else
    for j = 1 : nplaces
        for k = 1 : nlabels
            W(k,j) = log(1-Robots{i}.Beliefs{j}.Prob(k));
            W(k,j) = max(W(k,j),-1000);
        end
    end
end

if planner.ObsC
    %nu: transiciones que son supuestamente obstáculos (se evita ir por estas)
    nu = zeros(1,ntrans);
    for j = 1 : nplaces
        if Robots{i}.Beliefs{j}.Prob(nlabels-1)>planner.tau_obs
            [~,tran_obs] = find(PN.Post(j,:));
            nu(tran_obs) = 1;
        end
    end
    MILP_data.Aeq=[eye(nplaces),   -PN.C_incidence, zeros(nplaces,nlabels); ...
        zeros(1,nplaces), nu,             zeros(1,nlabels)];
    MILP_data.beq=[MILP_data.m0;0];
else
    MILP_data.Aeq=[eye(nplaces) -PN.C_incidence zeros(nplaces,nlabels)];
    MILP_data.beq=MILP_data.m0;
end

% Cost function
if planner.Pcost
    J_m = N*ones(1,nlabels)*W;
else
    J_m = zeros(1,nplaces);
end
if planner.Obscost
    J_sigma = ones(1,ntrans)-N*W(nlabels,:)*PN.Post;
else
    J_sigma = ones(1,ntrans);
end
J_x = zeros(1,nlabels);
MILP_data.f = [J_m, J_sigma, J_x];
% switch planner
%     case {1,2}
%         % Minimize only transitions
%         MILP_data.f=[zeros(1,nplaces) ones(1,ntrans) zeros(1,nlabels)];
%     case {3,4}
%         % Maximize the sum of probabilities
%         MILP_data.f=[N*ones(1,nlabels)*Robots{i}.W, ones(1,ntrans), zeros(1,nlabels)];
%     case {5,6,7}
%         % Penalize obstacles
%         MILP_data.f=[N*ones(1,nlabels)*Robots{i}.W, ones(1,ntrans)-N*Robots{i}.W(1,:)*Post, zeros(1,nlabels)];
% end

% Inequality constraints
[Atask,btask]=formula2constraints(char(formula), PN.nlabels);
if planner.Deterministic
    MILP_data.Aineq=[W  zeros(nlabels,ntrans)      -N*eye(nlabels);
        -W   zeros(nlabels,ntrans)      eye(nlabels);
        zeros(size(Atask,1),nplaces)     zeros(size(Atask,1),ntrans)      Atask];
    MILP_data.bineq=[zeros(nlabels*2,1);btask];
else
    tauP = log(1-planner.tau);
    if planner.b
        MILP_data.Aineq=[ -W,   zeros(nlabels,ntrans),      -N*eye(nlabels);   %for known environment Observation_matrix
            W,   zeros(nlabels,ntrans),       N*eye(nlabels);
            zeros(size(Atask,1),nplaces),     zeros(size(Atask,1),ntrans),      Atask
            zeros(nplaces,nplaces), PN.Post, zeros(nplaces,nlabels)];
        MILP_data.bineq=[-tauP*ones(nlabels,1);(N+tauP)*ones(nlabels,1);btask;ones(nplaces,1)];
    else
        MILP_data.Aineq=[ -W,   zeros(nlabels,ntrans),      -N*eye(nlabels);
            W,   zeros(nlabels,ntrans),       N*eye(nlabels);
            zeros(size(Atask,1),nplaces)     zeros(size(Atask,1),ntrans)      Atask];
        MILP_data.bineq=[-tauP*ones(nlabels,1);(N+tauP)*ones(nlabels,1);btask];
    end
end