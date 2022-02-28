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


function [active_observations, possible_regions, number_of_robots, marking_new, message] = rmt_check_active_observations(xmin,Pre,Post,m0,data,nplaces_orig,ntrans_orig,message)

m0_old = m0;
m0 = m0(1:nplaces_orig);
m0_Buchi = m0_old(nplaces_orig+2*length(data.Tr.props)+1:end);
m0_obs = m0_old(nplaces_orig+1:nplaces_orig+length(data.Tr.props));
message = sprintf('%s\n\n\t Initial state of Buchi = %s',message,mat2str(find(m0_Buchi)));
active_observations{1} = find(m0_obs);
if isempty(active_observations{1})
    message = sprintf('%s\n\t No active observations at initial state',message);
else
    message = sprintf('%s\n\t Active observations = %s',message,mat2str(active_observations{1}));
end
pos_regions={};
temp = find(m0);
marking_temp = zeros(length(temp));
for i = 1 : length(temp)
    pos_regions{i} = data.Tr.Cells{temp(i)};
    marking_temp(i) = m0(temp(i));
end
possible_regions{1} = pos_regions;
number_of_robots{1} = marking_temp;
flagF = 0; % flag to mark when the final state in Buchi was reached
% sum_transQ = 0; %count the fired transition in Quotient PN
sum_transB = 0; %count the real fired transition in Buchi PN 
for i = 1 : data.optim.paramWith.interM
%     if mod(i,2) == 1 && flagF == 0 % i is odd
%         trans_Q = find([xmin((i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+1: ...
%             (i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1) + ntrans_orig)] > eps*1e9);
%         sum_transQ = sum_transQ + length(trans_Q);
%     end
    
    if (i/2 == round(i/2))
        trans_buchi=find([xmin((i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+ntrans_orig+1:i*(size(Pre,1)+size(Pre,2)))] > eps*1e9);
        input_place = find(Pre(:,trans_buchi+ntrans_orig));
        input_place = input_place(input_place>nplaces_orig+2*length(data.Tr.props))-nplaces_orig-2*length(data.Tr.props); %take only the place of the buchi
        output_place = find(Post(:,trans_buchi+ntrans_orig));
        output_place = output_place(output_place>nplaces_orig+2*length(data.Tr.props))-nplaces_orig-2*length(data.Tr.props);

        if flagF == 0
        sum_transB = sum_transB + length(trans_buchi);
        end
        if find(data.B.F == output_place)
            flagF = 1;
        end
             message = sprintf('%s\n Transition in Buchi from state %d to state %d with observation (%s)',message,...
            input_place,output_place,mat2str(find([xmin((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+1:(i-1)*(size(Pre,1)+size(Pre,2))+length(data.Tr.props)+nplaces_orig)] > eps*1e9)));
        message = sprintf('%s\n\t State of Buchi in step %d = %s',message,i/2,...
            mat2str(find([xmin((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+2*length(data.Tr.props)+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1))])));
        
        active_observations{length(active_observations)+1} = find([xmin((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+1:(i-1)*(size(Pre,1)+size(Pre,2))+length(data.Tr.props)+nplaces_orig)] > eps*1e9);
        if isempty(active_observations{length(active_observations)})
            message = sprintf('%s\n\t No active observations at step %d',message,i/2);
        else
            message = sprintf('%s\n\t Active observations at step %d = %s',message,i/2,mat2str(active_observations{length(active_observations)}));
        end
        %take the new marking of the robot model
        marking_new = [xmin((i-1)*(size(Pre,1)+size(Pre,2))+1:(i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig)];
        temp = find(marking_new > eps*1e9);%marking of places modeling the team; condition > eps necessary for gplk solution
        pos_regions={};
        marking_temp = zeros(1,length(temp));
        for j = 1 : length(temp)
            pos_regions{j} = data.Tr.Cells{temp(j)};
            marking_temp(j) = marking_new(temp(j));
        end
        
        possible_regions{length(possible_regions)+1} = pos_regions;
        number_of_robots{length(number_of_robots)+1} = marking_temp; %number of robots in each macro region
        message = sprintf('%s\n\t Possible regions for the robots',message);
        for k = 1 : length(pos_regions)
            message = sprintf('%s\n\t\t --- %s',message,mat2str(pos_regions{k}));
        end
    end
end

message = sprintf('\n ****** %s ****** \n The sum of all the fired transitions in Buchi PN is: %d',message,sum_transB);

%remove eventually identical observations
for j = length(active_observations):-1:2
    if (isempty(setxor(active_observations{j},active_observations{j-1})) && ...
            isempty(setxor(unique([possible_regions{j}{:}]),unique([possible_regions{j-1}{:}]))))
        active_observations(j) = [];
        possible_regions(j) = [];
        number_of_robots(j) = [];
    end
end