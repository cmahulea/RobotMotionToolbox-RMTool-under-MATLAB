%    This is part of RMTool - Robot Motion Toolbox, for Matlab 2010b or newer.
%
%    Copyright (C) 2016 RMTool developing team. For people, details and citing 
%    information, please see: http://webdiis.unizar.es/RMTool/index.html.
%
%    This program is free software: you can redistribute it and/or modify
%    it under the terms of the GNU General Public License as published by
%    the Free Software Foundation, either version 3 of the License, or
%    (at your option) any later version.
%
%    This program is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%    GNU General Public License for more details.
%
%    You should have received a copy of the GNU General Public License
%    along with this program.  If not, see <http://www.gnu.org/licenses/>.

%% ============================================================================
%   MOBILE ROBOT TOOLBOX
%   Graphical User Interface
%   First version released on January, 2019. 
%   Last modification January 31, 2019.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================


function [varargout] = rmt_constraints_PN_obs_new(Pre, Post, m0, props, trans_buchi, obs_type, k, alpha, beta, gamma, solver)
%constraints for PN model based on desired observations
%construct linear constraints s.t. a set of observations is satisfied (in final state and, if desired, along trajectory)

%Pre,Post,m0 correspond to PN model

%props - N_p atomic propositions
%props{i} contains places (partition cells) that belong to proposition (region) i

%set_ind_fin - column vector with indices of observations to be satisfied in final marking
%set_ind_traj - column vector with indices of observations to be satisfied along trajectory
%set_ind_traj may not contain set_ind_fin, since trajectory is considered from m0 up to (excluding) final marking
%each observation is a row of observable set Obs
%thus, set_ind_fin (and set_ind_traj) contains indices for rows in Obs
%if any observation is good along trajectory, take " set_ind_traj=(1:size(Obs,1))' " (do not take set_ind_traj empty)
%for obs_type == 'final', set_ind_traj is not used (doesn't matter what is true along trajectory)

%obs_type imposes where and which observations should be satisfied:
%obs_type can be 'final','trajectory','intermediate';
%k is the number of intermediate markings (used if obs_type == 'intermediate')
%for any value of obs_type, the observations given by set_ind_fin are satisfied in final marking
%if obs_type == 'final'(default), we have k==1 (only final marking), and no restrictions on trajectory;
%if obs_type == 'trajectory', we have k ==1 and observations set_ind_traj are true in all reached markings on trajectory given by sigma;
%if obs_type == 'intermediate', we should have k > 1 and observations set_ind_traj are true in all intermediate k markings

%solver can be 'intlinprog' (Matlab's ILP), 'glpk' (external), or 'cplex' (external) - the output includes the arguments for running the chosen solver
%       if solver=='intlinprog', the function has outputs: [cost, intcon, A, b, Aeq, beq, lb, ub] (see intlincon's help); options can be choosen when running intlinprog
%       if solver=='cplex', the function has outputs: [cost, A, b, Aeq, beq, lb, ub, vartype] (see intlincon's help); options can be choosen when running intlinprog
%       if solver=='glpk', the function has outputs: [cost, matrix_A, vector_b, lb, ub, ctype, vartype, sense] (see glpk's help)

%alpha, beta, gamma are weights in cost for: 
%   alpha: number of fired transitions,
%   beta: number of moving robots, 
%   gamma: maximum number of robots that crossed any place (for collision avoidance)

%%***the ILP variables are:
%   m_i (size nplaces*k); sigma_i (size ntrans*k);
%   binary_variables for atomic props (size N*p or 2*N_p -first on final maring, last on traj.- last ones not used if obs_type=='final');
%   (currently only for ILP 1 and 2, not for that with k): w_i (size nplaces*k), positive vector for norm 1 of (m_0 - m_final); z_i (size 1*k), positive reals for norm inf. of (Pre*sigma+m_final).
%order of variables in vector with unknowns: m1 (vector), sigma_1 (vector), m_2, sigma_2, ..., sigma_k,  x_1, x_2,...,x_{N_p},   (x_1(t), x_2(t),...,x_{N_p}(t)),   w_1 (vector),z_1(scalar), w_2,z_2, ..., z_k.

%constraints to be defined have the form: A*X <= B, Aeq*X = Beq

nplaces = size(Pre,1); %number of places
ntrans = size(Pre,2); %number of transitions
N_r = sum(m0); % number of robots
N_p = length(props); %number of atomic props.
trans_buchi_final = trans_buchi{1}; %formula for the transition to the new state in buchi
trans_buchi_self = trans_buchi{2}; %formula for the selfloop on the initial state
M = min((N_r+1)^2,0.5e5); %big number for the optimization problems for higher values errors in optimization may result (very small x can lead x*M being equal to 1, thus wrongly satisfying a constraint)

if ~strcmp(obs_type,'intermediate')
    k=1;    %for 'final' or 'trajectory'
end


%%***add the state equation: m_j = m_{j-1} + (Post-Pre)*sigma_j,  j=1,...,k
    %for 'intermediate': additional constraints: %m_{j-1} - Pre \cdot sigma_j \geq 0,  j=1,...,k
Aeq = [eye(nplaces) , -(Post-Pre)];    %for m_1 = m0 + (Post-Pre)*sigma_1
beq = m0;
if strcmp(obs_type,'intermediate')
    A = [zeros(nplaces) , Pre]; %for m0 - Pre \cdot sigma \geq 0
    b = m0;
    
    for i = 2 : k %loop entered only for 'intermediate'
        Aeq = [Aeq , zeros(size(Aeq,1),nplaces+ntrans)]; %add nplaces+ntrans columns to Aeq
        Aeq = [Aeq; zeros(nplaces,(i-2)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) eye(nplaces) -(Post-Pre)]; %add the state equation
        beq = [beq;zeros(nplaces,1)];
        
        A = [A zeros(size(A,1),nplaces+ntrans)]; %add nplaces+ntrans columns to A
        A = [A ; zeros(nplaces, (i-2)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) zeros(nplaces,nplaces) Pre];
        b = [b ; zeros(nplaces,1)];
    end
end


%%***characteristic vector for each proposition
V = zeros(N_p,nplaces);    %characteristic vectors on rows of matrix V
for i = 1 : N_p
    V(i,props{i})=1; %characteristic vector of proposition i -> vi
end


%%***(F)define the boolean variables for final states (first N_p binary variables) and link them with final PN marking and with set to be observed
Aeq = [Aeq , zeros(size(Aeq,1),N_p)]; % add new columns to Aeq
A=[];   
b=[];

if (trans_buchi_final ~= Inf) %if transition in Buchi is "True", don't add constraints (leave unconstraint binary vars)
    %F.1)link boolean variables for final state with final PN marking: same for any obs_type ('intermediate','trajectory','final')
    %xi=1 if proposition i is true at last marking
    %vi*(m_final) \leq M*xi   , forall i=1,...,N_p
    A = [A ; [ zeros(N_p,(k-1)*(nplaces+ntrans)) , V , zeros(N_p,ntrans) , -M*eye(N_p) ]];  %all first N_p constraints: V for final marking, eye for big_M
    b = [b ; zeros(N_p,1)];
    %vi*(m_final) \geq xi   , forall i=1,...,N_p
    A = [A ; [ zeros(N_p,(k-1)*(nplaces+ntrans)) , -V , zeros(N_p,ntrans) , eye(N_p) ]];
    b = [b ; zeros(N_p,1)];
    
    %F.2)link boolean variables for final state with boolean formula on the transition from Buchi: same for any obs_type ('intermediate','trajectory','final')
    
    boolean_formula='';
    indices = find(trans_buchi_final>0);
    for i = 1: length(indices)-1
        boolean_formula = sprintf('%s Y%d &',boolean_formula,trans_buchi_final(indices(i)));
    end
    if (length(indices)>=1)
        boolean_formula = sprintf('%s Y%d',boolean_formula,trans_buchi_final(indices(length(indices))));
    end
    indices = find(trans_buchi_final < 0);
    if ((length(indices) >= 1) && ~isempty(boolean_formula))
        boolean_formula = sprintf('%s &',boolean_formula);
    end
    for i = 1: length(indices)-1
        boolean_formula = sprintf('%s !Y%d &',boolean_formula,-trans_buchi_final(indices(i)));
    end
    if (length(indices)>=1)
        boolean_formula = sprintf('%s !Y%d',boolean_formula,-trans_buchi_final(indices(length(indices))));
    end
    [~,~,~,Ar,br] = rmt_formula2constraints(boolean_formula, [],[],length(props));
    Ar = Ar(1:size(Ar,1),1:length(props));

    A = [A ; [ zeros(size(Ar,1),k*(nplaces+ntrans)) , Ar ]];  %all first N_p constraints: V for final marking, eye for big_M
    b = [b ; br];
end


%%***(T)define the boolean variables for trajectory (last N_p binary variables) and link them with PN markings and with set to be observed
%T.1)link boolean variables for trajectory with PN markings

if strcmp(obs_type,'trajectory')    %xi=1 if proposition i was true at least once along all reached markings (trajectory)
    Aeq = [Aeq , zeros(size(Aeq,1),N_p)]; % add new columns to Aeq
    A = [A , zeros(size(A,1),N_p)]; % add new columns to A
    if (isempty(trans_buchi_self)) %if there is no selfloop transition in Buchi compute add contraint on the empty
        not_null_obs = [];
        act_obst_cellsm0 = [];
        V_act_obs_init = zeros(1,nplaces);
        for i = 1 : length(props)
            not_null_obs = union(not_null_obs,props{i});
            obs_m0 = intersect(find(m0),props{i}); 
            if ~isempty(obs_m0) %check if the initial pose of the robots activate any observation
                act_obst_cellsm0 = [act_obst_cellsm0 obs_m0'];
                V_act_obs_init = V_act_obs_init + V(i,:);
            end
        end
        null_obs = setdiff([1:nplaces],not_null_obs);
        V_null = zeros(1,nplaces);    %places corresponding to the empty space (placeses with null observations)
        V_null(null_obs) = 1;
        
        if intersect(find(m0),null_obs) %if at least one robot is in the free space, update V_act_obs_init with the free observation
            V_act_obs_init = V_act_obs_init + V_null;
        end
        % Post*sigma \leq V_null - during the trajectory only through empty
        % space is possible to pass
        
        if (isempty(setdiff(find(m0),null_obs))) %if no selfloop and no observation active put that during the trajectory only empty observation is possible 
            A = [A ; [ zeros(nplaces,nplaces) , Pre , zeros(nplaces,2*N_p) ]];
            b = [b ; V_null'];
            need_check_ILP2=0;
        else %only the initial active observations are possible based on robots position
            A = [A ; [ zeros(nplaces,nplaces) , Pre , zeros(nplaces,2*N_p)]];
            b = [b ; V_act_obs_init'];
            need_check_ILP2=1;
        end
        
        
    elseif (trans_buchi_self ~= Inf) %if the selfloop transition in Buchi is "True", don't add constraints (leave unconstraint binary vars)
        %link boolean variables for trajectory with reached PN markings given by Post*sigma+m0-mf:
        %vi*(Post*sigma +m0-mf) \leq M*xi   , forall i=1,...,N_p
        A = [A ; [ -V , V*Post , zeros(N_p) , -M*eye(N_p) ]];  %all first N_p constraints: -V for m_final,  V*Post for sigma , zeros for final binary vars, eye for big_M
        b = [b ; -V*m0];
        % xi \leq vi*(Post*sigma +m0-mf)  , forall i=1,...,N_p
        A = [A ; [ V , -V*Post , zeros(N_p) , eye(N_p) ]];
        b = [b ; V*m0];
        
        boolean_formula='';
        indices = find(trans_buchi_self>0);
        for i = 1: length(indices)-1
            boolean_formula = sprintf('%s Y%d &',boolean_formula,trans_buchi_self(indices(i)));
        end
        if (length(indices)>=1)
            boolean_formula = sprintf('%s Y%d',boolean_formula,trans_buchi_self(indices(length(indices))));
        end
        indices = find(trans_buchi_self < 0);
        if ((length(indices) >= 1) && ~isempty(boolean_formula))
            boolean_formula = sprintf('%s &',boolean_formula);
        end
        for i = 1: length(indices)-1
            boolean_formula = sprintf('%s !Y%d &',boolean_formula,-trans_buchi_self(indices(i)));
        end
        if (length(indices)>=1)
            boolean_formula = sprintf('%s !Y%d',boolean_formula,-trans_buchi_self(indices(length(indices))));
        end
        [~,~,~,Ar,br] = rmt_formula2constraints(boolean_formula, [],[],length(props));
        Ar = Ar(1:size(Ar,1),1:length(props));
        
        A = [A ; [ zeros(size(Ar,1),k*(nplaces+ntrans)), zeros(size(Ar,1),N_p) , Ar ]];  %all first N_p constraints: V for final marking, eye for big_M
        b = [b ; br];
    end

    
end


%%***variables and constraints for w_i (size nplaces*k), z_i (size 1*k) for cost function - norm 1, norm infinity
if ~isempty(Aeq)  %if Aeq was empty, leave it as is to avoid warnings
    Aeq = [Aeq , zeros(size(Aeq,1),k*(nplaces+1))]; % add new columns to Aeq
end
if ~isempty(A)  %if A was empty, leave it as is to avoid warnings
    A = [A , zeros(size(A,1),k*(nplaces+1))]; %add new columns to A
end
%order of variables in vector with unknowns: m1 (vector), sigma_1 (vector), m_2, sigma_2, ..., sigma_k,  x_1, x_2,...,x_{N_p},   (x_1(t), x_2(t),...,x_{N_p}(t)),   w_1 (vector),z_1(scalar), w_2,z_2, ..., z_k.
if strcmp(obs_type,'trajectory')  %2*N_p binary vars, k=1
    % n_prev_vars=k*(nplaces+ntrans)+2*N_p; %number of previous variables
    A = [A ; [ -eye(nplaces) , zeros(nplaces,ntrans+2*N_p) , -eye(nplaces) , zeros(nplaces,1) ] ]; %m0-m_fin <= w
    b = [b ; -m0];
    A = [A ; [  eye(nplaces) , zeros(nplaces,ntrans+2*N_p) , -eye(nplaces) , zeros(nplaces,1) ] ]; %-m0+m_fin <= w
    b = [b ; m0];
    
    A = [A ; [ eye(nplaces) , Pre , zeros(nplaces,2*N_p+nplaces) , -ones(nplaces,1) ] ]; %m_fin + Pre*sigma <= z*ones(nplaces,1) 
    b = [b ; zeros(nplaces,1)];
elseif strcmp(obs_type,'intermediate') %2*N_p binary vars, k>1
    fprintf('\n New cost function not yet implemented for intermediate case!\n');
    %     m_k indices in unknowns vector:  (k-1)*(nplaces+ntrans)+1 : (k-1)*(nplaces+ntrans)+nplaces
else %case 'final' is default
    % n_prev_vars=k*(nplaces+ntrans)+N_p; %number of previous variables
    A = [A ; [ -eye(nplaces) , zeros(nplaces,ntrans+N_p) , -eye(nplaces) , zeros(nplaces,1) ] ]; %m0-m_fin <= w
    b = [b ; -m0];
    A = [A ; [  eye(nplaces) , zeros(nplaces,ntrans+N_p) , -eye(nplaces) , zeros(nplaces,1) ] ]; %-m0+m_fin <= w
    b = [b ; m0];  
    
    A = [A ; [ eye(nplaces) , Pre , zeros(nplaces,N_p+nplaces) , -ones(nplaces,1) ] ]; %m_fin + Pre*sigma <= z*ones(nplaces,1) 
    b = [b ; zeros(nplaces,1)];
end


%%***create cost function (column vector), other data for running glpk or intlinprog
if (strcmp(obs_type,'intermediate') || strcmp(obs_type,'trajectory'))   %2*N_p binary vars
%    cost = [ repmat([zeros(1,nplaces),alpha*ones(1,ntrans)] , 1,k) , zeros(1,2*N_p), beta*ones(1,nplaces) gamma]';    %variables: k*(nplaces+ntrans) integer; 2*N_p binary
    cost = [];
    for i = 1 : k
        cost = [ cost, [zeros(1,nplaces),alpha*i*ones(1,ntrans)] , zeros(1,2*N_p), beta*i*ones(1,nplaces) gamma]';    %variables: k*(nplaces+ntrans) integer; 2*N_p binary
    end
    lb=zeros( k*(nplaces+ntrans) + 2*N_p + k*(nplaces+1), 1);   %lower bounds
    %upper bounds used only for Matlab's intlinprog: only for binary vars
else %case 'final' is default
%    cost = [ repmat([zeros(1,nplaces),alpha*ones(1,ntrans)] , 1,k) , zeros(1,N_p), beta*ones(1,nplaces) gamma]';    %variables: k*(nplaces+ntrans) integer; N_p binary
    cost = [];
    for i = 1 : k
        cost = [ cost, [zeros(1,nplaces),alpha*i*ones(1,ntrans)] , zeros(1,N_p), beta*i*ones(1,nplaces) gamma]';    %variables: k*(nplaces+ntrans) integer; N_p binary
    end
    lb=zeros( k*(nplaces+ntrans) + N_p  + k*(nplaces+1), 1);   %lower bounds
end


%%***put data in desired form for indicated solver
switch solver
    case 'glpk' %outputs: [cost, matrix_A, vector_b, lb, ub, ctype, vartype, sense]
        ctype = [repmat('S',1,size(Aeq,1)) , repmat('U',1,size(A,1))];  %A*X<=b ('U') and Aeq*X=beq ('S');
        m_sig = [repmat('C',1,nplaces) , repmat('I',1,ntrans)]; %vartype: markings real ('C') - guaranteed by integer sigma, sigma integers ('I'), binary ('B')
        if (strcmp(obs_type,'intermediate') || strcmp(obs_type,'trajectory'))   %constraint and variable types
            vartype = [repmat(m_sig,1,k) , repmat('B',1,2*N_p) , repmat('C',1,k*(nplaces+1))];  %markings,w,z real ('C'), sigma integers ('I'), binary ('B')
        else %case 'final' is default
            vartype = [repmat(m_sig,1,k) , repmat('B',1,N_p) , repmat('C',1,k*(nplaces+1))];  %markings,w,z real ('C'), sigma integers ('I'), binary ('B')
        end
        varargout{1}=cost;
        varargout{2}=[Aeq;A];
        varargout{3}=[beq;b];
        varargout{4}=lb;
        varargout{5}=[];    %instead of upper bound, some vars are set binary
        varargout{6}=ctype;
        varargout{7}=vartype;
        varargout{8}=1; %minimization

    case 'intlinprog' %outputs: [cost,intcon,A,b,Aeq,beq,lb,ub]
        intcon = find(repmat([zeros(1,nplaces) , ones(1,ntrans)],1,k));  %set as integer sigma_j (m_j can be real)
        if (strcmp(obs_type,'intermediate') || strcmp(obs_type,'trajectory'))   %2*N_p binary vars
            intcon = [intcon , (k*(nplaces+ntrans)+1):(k*(nplaces+ntrans)+2*N_p)];  %2*N_p binary vars
            ub = [ Inf(k*(nplaces+ntrans),1) ; ones(2*N_p,1) ; Inf(k*(nplaces+1),1) ];   %upper bounds only for binary vars
        else %case 'final' is default
            intcon = [intcon , (k*(nplaces+ntrans)+1):(k*(nplaces+ntrans)+N_p)];  %N_p binary vars
            ub=[ Inf(k*(nplaces+ntrans),1) ; ones(N_p,1) ; Inf(k*(nplaces+1),1) ];
        end
        varargout{1}=cost;
        varargout{2}=intcon;
        varargout{3}=A;
        varargout{4}=b;
        varargout{5}=Aeq;
        varargout{6}=beq;
        varargout{7}=lb;
        varargout{8}=ub;

    case 'cplex' %outputs: [cost,A,b,Aeq,beq,lb,ub,vartype]
        %cplex's vartype can contain: 'B','I','C','S',or 'N': binary, general integer, continuous, semi-continuous(?) or semi-integer(?)
        m_sig = [repmat('C',1,nplaces) , repmat('I',1,ntrans)]; %vartype: markings real ('C') - guaranteed by integer sigma, sigma integers ('I'), binary ('B')
        if (strcmp(obs_type,'intermediate') || strcmp(obs_type,'trajectory'))   %constraint and variable types
            vartype = [repmat(m_sig,1,k) , repmat('B',1,2*N_p) , repmat('C',1,k*(nplaces+1))];  %markings,w,z real ('C'), sigma integers ('I'), binary ('B')
        else %case 'final' is default
            vartype = [repmat(m_sig,1,k) , repmat('B',1,N_p) , repmat('C',1,k*(nplaces+1))];  %markings,w,z real ('C'), sigma integers ('I'), binary ('B')
        end
        varargout{1}=cost;
        varargout{2}=A;   %A*X<=b;
        varargout{3}=b;
        varargout{4}=Aeq;   %Aeq*X=beq
        varargout{5}=beq;
        varargout{6}=lb;
        varargout{7}=[];    %instead of upper bound, some vars are set binary
        varargout{8}=vartype;
        %varargout{9}=need_check_ILP2;
    
    otherwise
        fprintf('\nERROR when constructing PN constraints: unknown or wrong indicated solver.\n')
end
