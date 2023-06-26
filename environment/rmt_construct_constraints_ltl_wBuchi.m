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
%   First version released on September, 2014. 
%   Last modification Enero, 2020.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function [A,b,Aeq,beq,cost] = rmt_construct_constraints_ltl_wBuchi(Pre,Post,m0, ntrans_orig, U, final, idxV, varargin)

% U = 2*k intermediate markings (in the even/odd steps, the transitions are
% fired in Quotient/Buchi PN models
flag_sisf = 0;
flag_suf = 0; % flag_suf = 1 only when the suffix is computed, otherwise is 0;
bad_sol = []; % (teta in paper) --> matrix containing all the bad solutions (markings) from previous iterations on each column, as a result of MILP based on the reduced PN model, which could not be projected
if nargin == 8
    bad_sol = varargin{1};
elseif nargin == 9
    bad_sol = varargin{1};
    flag_sisf = varargin{2};% used to compute the prefix, so that the first fired transition is not a virtual one
else
    flag_suf = 1;
end


trans_model = [zeros(1,length(m0)) ones(1,ntrans_orig) zeros(1,size(Pre,2)-ntrans_orig)];
idxV_Buchi = idxV - ntrans_orig;
trans_for_Buchi = ones(1,size(Pre,2)-ntrans_orig);
trans_Buchi = [zeros(1,length(m0)) zeros(1,ntrans_orig) trans_for_Buchi]; % trans Buchi (real and virtual)

aux_for_Buchi = trans_for_Buchi; 
aux_for_Buchi(idxV_Buchi) = 0;
temp_vect_it = [zeros(1,length(m0) + ntrans_orig + length(trans_for_Buchi))];  % dummy temp vector used in the constraints of the binary variables
real_trans = [ones(1,size(Pre,2) - length(idxV))]'; % all the real transitions in the model, without the virtual ones
marking_final = zeros(1,length(m0));
marking_final(final)=1;

%the variables are: [m_i sigma_i z w], i = 1,...,U, z, w are binary vectors
%for the decision variables necessary, where z and v each have the size
%(ntrans_real,1) and ensure that the new solution is different than the old
%ones. The pair (z,w) is added for each old solution.

%size of unknown variables z,w 
ntrans_real = size(real_trans,1);

nplaces = size(Pre,1); %number of places
ntrans = size(Pre,2); % number of transitions

% a), b)
%add the state equation: m_{i+1} = m_i + (Post-Pre)*sigma_{i+1}
Aeq = [eye(nplaces) -(Post-Pre)]; % state equation for m_1 = m0 + (Post-Pre)*sigma_1
beq = m0;
A = [zeros(nplaces,nplaces) Pre]; %m0 - Pre \cdot sigma \geq 0
b = m0;
%in the first step fire only transitions of the robot model
Aeq = [Aeq; trans_Buchi];
beq = [beq;0];

for i = 2 : U
    Aeq = [Aeq zeros(size(Aeq,1),nplaces+ntrans)]; %add nplaces+ntrans columns to Aeq
    Aeq = [Aeq; zeros(nplaces,(i-2)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) eye(nplaces) -(Post-Pre)]; %add the state equation
    beq = [beq;zeros(nplaces,1)];
    % c), d)
    if (i/2 == round(i/2)) %fire only transitions of the Buchi automaton
        Aeq = [Aeq; zeros(1,(i-1)*(nplaces+ntrans)) trans_model]; %not fire transition of the model
        beq = [beq;0];
        if flag_suf == 1
         if i == 2% j) force one transition in buchi from final state., but not the virtual one
            
            trans_for_Buchi(idxV_Buchi) = 0;
            trans_BuchiB = [zeros(1,length(m0)) zeros(1,ntrans_orig) trans_for_Buchi];
         else
            trans_BuchiB = trans_Buchi;
         end
        else 
            if i == 2 && flag_sisf == 0 % j) force one transition in buchi from final state., but not the virtual one
                
                trans_for_Buchi(idxV_Buchi) = 0;
                trans_BuchiB = [zeros(1,length(m0)) zeros(1,ntrans_orig) trans_for_Buchi];
            else
                trans_BuchiB = trans_Buchi;
                
            end
        end
            Aeq = [Aeq; zeros(1,(i-1)*(nplaces+ntrans)) trans_BuchiB]; %fire one transition of Buchi
            beq = [beq;1];
        
    % f)     
    else %fire only transitions of the robot model
        Aeq = [Aeq; zeros(1,(i-1)*(nplaces+ntrans)) trans_Buchi];
        beq = [beq;0];
    end
    A = [A zeros(size(A,1),nplaces+ntrans)]; %add nplaces+ntrans columns to A
    A = [A ; zeros(nplaces, (i-2)*(nplaces+ntrans)) -eye(nplaces) zeros(nplaces,ntrans) zeros(nplaces,nplaces) Pre];
    b = [b ; zeros(nplaces,1)];
end

% this should not be used 
% % %put that an intermediate marking of Buchi is equal with a final one in the final
% % interm = round(k/2);
% % % m(interm) == m_final
% % 
% % nplaces_buchi = nplaces-nplaces_orig-nplaces_observ;
% % 
% % Aeq=[Aeq; zeros(nplaces_buchi,(interm-1)*(nplaces+ntrans)) zeros(nplaces_buchi,nplaces_orig+nplaces_observ) eye(nplaces_buchi) ...
% %     zeros(nplaces_buchi,ntrans) zeros(nplaces_buchi,(k-interm-1)*(nplaces+ntrans)) zeros(nplaces_buchi,nplaces_orig+nplaces_observ) ...
% %     -eye(nplaces_buchi) zeros(nplaces_buchi,ntrans)];
% % beq = [beq ; zeros(nplaces_buchi,1)];

% g) new f*mB = 1
% -m_final * 1 <=-1
% final marking in Buchi needs to be equal with 1
Aeq = [Aeq; zeros(1,(U-1)*(nplaces+ntrans)) marking_final zeros(1,ntrans)];
beq = [beq;1];

% extend matrice A, Aeq, with the new unknown variables z, w
A = [A zeros(size(A,1),2*ntrans_real*size(bad_sol,2))];
Aeq = [Aeq zeros(size(Aeq,1),2*ntrans_real*size(bad_sol,2))];


% add constraints for the binary variables to ensure that the new solution
% if different than the old ones
if ~isempty(bad_sol)
    aux_size = length(temp_vect_it);
    M = U + 1;
    for i = 1:size(bad_sol,2)
        for j = 1:U
            % z[l] = 1 if old solution teta[l] (in position l) is different (<0 = <= -1) than the
            % current solution sum(real_trans)[l], 0 otherwise
            
            A = [A; zeros(ntrans_real,j*nplaces + (j-1)*(length(idxV)+ntrans_real))...
                eye(ntrans_real) zeros(ntrans_real,(U-j)*(nplaces + ntrans_real) + (U-j+1)*length(idxV))...
                repmat(zeros(ntrans_real,2*ntrans_real),[1, i-1]) M*eye(ntrans_real) zeros(ntrans_real)...
                repmat(zeros(ntrans_real,2*ntrans_real),[1, size(bad_sol,2)-i])];
            b = [b; (M-1)*ones(ntrans_real,1)+bad_sol(:,i)];
            
            A = [A; zeros(ntrans_real,j*nplaces + (j-1)*(length(idxV)+ntrans_real))...
                -eye(ntrans_real) zeros(ntrans_real,(U-j)*(nplaces + ntrans_real) + (U-j+1)*length(idxV))...
                repmat(zeros(ntrans_real,2*ntrans_real),[1, i-1]) -M*eye(ntrans_real) zeros(ntrans_real)...
                repmat(zeros(ntrans_real,2*ntrans_real),[1, size(bad_sol,2)-i])];
            b = [b; (1-eps*10^5)*ones(ntrans_real,1)-bad_sol(:,i)];
            
            
            % w[l] = 1 if old solution teta[l] (in position l) is different (>0 = >= 1) than the
            % current solution sum(real_trans)[l], 0 otherwise
            
            A = [A; zeros(ntrans_real,j*nplaces + (j-1)*(length(idxV)+ntrans_real))...
                -eye(ntrans_real) zeros(ntrans_real,(U-j)*(nplaces + ntrans_real) + (U-j+1)*length(idxV))...
                repmat(zeros(ntrans_real,2*ntrans_real),[1, i-1]) zeros(ntrans_real) M*eye(ntrans_real)...
                repmat(zeros(ntrans_real,2*ntrans_real),[1, size(bad_sol,2)-i])];
            b = [b; (M-1)*ones(ntrans_real,1)-bad_sol(:,i)];
            
            A = [A; zeros(ntrans_real,j*nplaces + (j-1)*(length(idxV)+ntrans_real))...
                eye(ntrans_real) zeros(ntrans_real,(U-j)*(nplaces + ntrans_real) + (U-j+1)*length(idxV))...
                repmat(zeros(ntrans_real,2*ntrans_real),[1, i-1]) zeros(ntrans_real) -M*eye(ntrans_real)...
                repmat(zeros(ntrans_real,2*ntrans_real),[1, size(bad_sol,2)-i])];
            b = [b; (1-eps*10^5)*ones(ntrans_real,1)+bad_sol(:,i)];
        end
        
        % sum(z) + sum(w) >= 1
        A = [A; zeros(1,aux_size*U) repmat(zeros(1,2*ntrans_real),[1,i-1]) -ones(1,2*ntrans_real)...
            repmat(zeros(1,2*ntrans_real),[1,size(bad_sol,2)-i])];
        b = [b;-1];
        
    end
    
end


%move at least one robot in the first step
% A = [A; zeros(1,nplaces) -trans_model zeros(1,(k-1)*(nplaces+ntrans))];
% b = [b ; -1];

%fire at lesat one transiton of the Buchi in last step 
%A = [A; zeros(1,(k-1)*(nplaces+ntrans)) -trans_Buchi];
%b = [b ; -1];

%fprintf(1,'\nIntermediate states %d',interm);
%%%%%%%%%%% cost function
cost = [];
trans_QB = [ones(1,ntrans_orig) trans_for_Buchi]; % the cost function will consider only the real transitions
for i = 1 : U
    cost = [cost zeros(1,nplaces) i*trans_QB];
end

cost = [cost zeros(1,2*ntrans_real*size(bad_sol,2))];



