%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  draw_BeliefMapsInitial
%   Eduardo Montijano
%   Date: March 2021
%   Function for the distributed estimation CASE 2021 + Robotarium
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Robots = updateBeliefs(Robots,T,W,EstimationType)

N_r = length(Robots);
switch EstimationType
    case 1 %Each robot acts independently
        for i=1:N_r
            for x=1:length(T.Q)
                prior = Robots{i}.Beliefs{x}.Prob;
                Likelyhood = Robots{i}.Observation{x}.Likelyhood;
                scale = Likelyhood'*prior;
                posterior = prior.*Likelyhood/scale;
                Robots{i}.Beliefs{x}.Prob = posterior;
                % Update the most probable region
                if (max(Robots{i}.Beliefs{x}.Prob) == Robots{i}.Beliefs{x}.Prob(length(T.props)+1))%if more regions have the same probability including the empty observation (free space), consider the free space
                    Region = length(T.props)+1;
                else
                    [~,Region]= max(Robots{i}.Beliefs{x}.Prob);
                end
                Robots{i}.Beliefs{x}.Region = Region;
            end
        end
    case 2 %Centralized (All robots have the same estimation)
        for x=1:length(T.Q)
            prior = Robots{1}.Beliefs{x}.Prob;
            % Compute the likelihood considering all robots likelihoods
            Likelyhood = ones(length(T.props)+1,1);
            for i=1:N_r
                Likelyhood = Likelyhood.*Robots{i}.Observation{x}.Likelyhood;
            end
            scale = Likelyhood'*prior;
            posterior = prior.*Likelyhood/scale;
            for i=1:N_r
                Robots{i}.Beliefs{x}.Prob = posterior;
                % Update the most probable region
                if (max(Robots{i}.Beliefs{x}.Prob) == Robots{i}.Beliefs{x}.Prob(length(T.props)+1))%if more regions have the same probability including the empty observation (free space), consider the free space
                    Region = length(T.props)+1;
                else
                    [~,Region]= max(Robots{i}.Beliefs{x}.Prob);
                end
                Robots{i}.Beliefs{x}.Region = Region;
            end
        end
    case 3 %Consensus
        for x=1:length(T.Q)
            pi0 = zeros(length(T.props)+1,N_r);
            for i=1:N_r
                pi0(:,i) = Robots{i}.Observation{x}.Likelyhood;
            end
            %% Accelerated consensus
            ConsensusNumRounds = N_r;
            lm = -0.5;
            lM = 0.5;
            c = 2/(lM-lm);
            d = (lM+lm)/(lM-lm);
            a0 = 1;
            a1 = c-d;
            x0T = log(pi0)';
            x0T(isinf(x0T))=-1e99;
            xavg = mean(x0T);
            x1T = (c*W*x0T-d*x0T)/a1;
            E1 = zeros(ConsensusNumRounds,1);
            for it=1:ConsensusNumRounds
                a2 = 2*(c-d)*a1-a0;
                x2T = 2*a1/a2*(c*W*x1T-d*x1T)-a0/a2*x0T;
                a0 = a1;
                a1 = a2;
                x0T = x1T;
                x1T = x2T;
                E1(it)=norm(repmat(xavg,N_r,1)-x2T);
            end
            pik = exp(x2T)';
            for i=1:N_r
                prior = Robots{i}.Beliefs{x}.Prob;
                Likelyhood = (pik(:,i).^N_r)/sum(pik(:,i).^N_r);
                scale = Likelyhood'*prior;
                posterior = prior.*Likelyhood/scale;
                Robots{i}.Beliefs{x}.Prob = posterior;
                
                
%                 pik(:,i) = (pik(:,i).^N_r)/sum(pik(:,i).^N_r);
%                 Likelyhood = pik(:,i);
% %                 Robots{i}.Observation{x}.Likelyhood = pik(:,i);
%                 scale = Likelyhood'*Robots{i}.Beliefs{x}.Prob;
%                 posterior = Robots{i}.Beliefs{x}.Prob.*Likelyhood/scale;
%                 belief = zeros(length(T.props)+1,1);
%                 for k=1:length(T.props)+1
%                     belief(k) = ProbXk1Xk(k,:)*posterior;
%                 end
%                 belief = belief/sum(belief);
%                 Robots{i}.Beliefs{x}.Prob = belief;
                % Update the most probable region
                if (max(Robots{i}.Beliefs{x}.Prob) == Robots{i}.Beliefs{x}.Prob(length(T.props)+1))%if more regions have the same probability including the empty observation (free space), consider the free space
                    Region = length(T.props)+1; %Considerar vacio
                    %Region = length(T.props); %Considerar obstaculo, mal,
                    %todo se vuelve de color negro
                else
                    [~,Region]= max(Robots{i}.Beliefs{x}.Prob);
                end
                Robots{i}.Beliefs{x}.Region = Region;
            end
        end
    case 4 % No update, keep the same beliefs
        %Robots{i}.Beliefs{x}.Prob
        %Robots{i}.Beliefs{x}.Region = Region
        % for all i and all x
end