
load('test_MILP.mat')

nplaces = size(Pre,1); %number of places
ntrans = size(Pre,2); % number of transitions
No_r = size(Run_cells,1);
mf = zeros(nplaces,1);
mf(Run_cells(:,end)) = 1; % final marking in PN


% Pre = zeros(5,8);
% Post = zeros(5,8);
% C=[-1 1 0 0 0 0 0 0;...
%     1 -1 -1 1 0 0 0 0;...
%     0 0 1 -1 -1 1 0 0;...
%     0 0 0 0 0 0 1 -1;...
%     0 0 0 0 1 -1 -1 1];
% Post = [0 1 0 0 0 0 0 0;...
%     1 0 0 1 0 0 0 0;...
%     0 0 1 0 0 1 0 0;...
%     0 0 0 0 0 0 1 0;...
%     0 0 0 0 1 0 0 1];

C = Post - Pre;
% m0 = [1 1 0  0 0]';
% mf = [0 0 0  1 1]';
%the variables are: [m_0^1 m_1 sigma_1 m_0^2 m_2 sigma_2 ... m_0^|R| m_|R| sigma_|R|]
Aeq = [];
beq = [];
A = [];
b = [];
% nplaces = 5;
% ntrans = 8;
% No_r = 2;
temp_zeros = zeros(nplaces,2*nplaces + ntrans);
temp_mi = [zeros(nplaces) eye(nplaces) zeros(nplaces,ntrans)]; % is considered only the variable m_i
temp_m0 = [eye(nplaces) zeros(nplaces) zeros(nplaces, ntrans)]; % is considered only the variable m0^i
temp_m0_single = [ones(1,nplaces) zeros(1,nplaces), zeros(1,ntrans)];
temp_mi_single = [ zeros(1,nplaces) ones(1,nplaces) zeros(1,ntrans)];
tic
for i = 1:No_r

    % a) state equation
    Aeq = [Aeq; repmat(temp_zeros,[1, i-1]) eye(nplaces) -eye(nplaces) C repmat(temp_zeros,[1, No_r - i])];
    beq = [beq; zeros(nplaces,1)];

    % b)
    if i < No_r
        A = [A; repmat(temp_mi,[1,i-1]) zeros(nplaces,2*nplaces) Post repmat(temp_m0,[1, No_r - i])];
        b = [b; ones(nplaces,1)];
    else 
       A = [A; repmat(temp_mi,[1,i-1]) zeros(nplaces,2*nplaces) Post];
       b = [b; ones(nplaces,1)];
    end

    % e)
    Aeq = [Aeq; repmat(zeros(1,2*nplaces+ntrans),[1, i-1]) temp_m0_single repmat(zeros(1,2*nplaces+ntrans),[1, No_r - i])];
    beq = [beq; ones(1,1)];

    % f)
    Aeq = [Aeq; repmat(zeros(1,2*nplaces+ntrans),[1, i-1]) temp_mi_single repmat(zeros(1,2*nplaces+ntrans),[1, No_r - i])];
    beq = [beq; ones(1,1)];

end

% c)
Aeq = [Aeq; repmat(temp_m0, [1,No_r])];
beq = [beq; m0];

% d)
Aeq = [Aeq; repmat(temp_mi, [1,No_r])];
beq = [beq; mf];

cost = [];
for i = 1:No_r
    cost = [cost zeros(1,2*nplaces) ones(1,ntrans)];
end
toc

vartype = '';
for r = 1:No_r

    for j = 1 : 2*nplaces
        vartype = sprintf('%sC',vartype); %put the markings as real
    end
    for j = 1 : ntrans
        vartype = sprintf('%sI',vartype); %put the sigma as integer
    end
end

tic
[xmin,~,exitflag] = cplexmilp(cost,A,b,Aeq,beq,[],[],[],zeros(1,size(A,2)),[],vartype);
toc
