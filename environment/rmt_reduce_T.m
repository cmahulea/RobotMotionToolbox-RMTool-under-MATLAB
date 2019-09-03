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
%   Last modification December 29, 2015.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function [T_r,propositions_r,R0_r] = rmt_reduce_T(T,propositions,R0)
%T_r: the reduced transition system model for 1 robot, by fusing(collapsing) adjacent cells with same observation
%propositions_r contains vectors with new labels (states from T_r) for the propositions
%R0_r is the mapped vector R0 through state reduction

%use parts from PN_Boolean functions (triangular_decomposition_regions.m, create_partition.m) for adapting T for argument of quotient_T.m
N_p=length(propositions);    %number of defined atomic props.
N_s=length(T.Q);    %number of initial states
OBS_set=[]; %will be a matrix with N_p columns, giving on rows the possible observations. A row like [1 3 0...0] means that current observation is only regions 1 and 3
%OBS_set will be sorted, and last row will correspond to free space, being [(N_p+1) 0 ... 0]
obs=zeros(1,N_s); %observations of each cell, as indices for rows of OBS_set
Sat_reg=zeros(N_s , N_p); %on row k we store satisfied regs of cell k
for k=1:N_s
    in_reg=zeros(1,length(propositions));    %will be 1 for regions containing current triangle (may be multiple ones)
    for i=1:length(propositions) %for each obstacle
        if ismember(k,propositions{i})      %current triangle is inside region i
            in_reg(i)=1;
        end
    end
    satisf_reg=find(in_reg);    %satisfied regs, in form [1 3] if only regions 1 & 3 are satisfied
    if isempty(satisf_reg)   %no satisfied reg. (add a dummy prop and pad with zeros)
        satisf_reg=zeros(1,N_p);
        satisf_reg(1)=N_p+1;    %index for dummy region (free space)
    elseif length(satisf_reg)<N_p   %pad with zeros until number of regions
        satisf_reg((end+1):N_p)=0;
    end     %satisf_reg has N_p value, having form [1 3 0...0]
    Sat_reg(k,:)=satisf_reg;    %store observation (do not compute row index for OBS_set yet, because OBS_set wil be sorted)
    OBS_set=[OBS_set ; satisf_reg];    %add current observation (may already be there)  (unique([OBS_set ; satisf_reg] , 'rows') would sort and keep unique, but it may be slower reiterated)
end
OBS_set = unique(OBS_set,'rows');   %keep unique rows and sort them (OBS_set contains only possible overlappings of regions)
for k=1:N_s   %now find observable of each triangle
    [~,obs(k)]=ismember(Sat_reg(k,:),OBS_set,'rows');    %observable
end


T.OBS_set=OBS_set;
T.obs=obs;
% %find cells corresponding to each defined region (proposition): 
T.props=propositions;
% T.props=cell(1,length(propositions));  %props{i} will be row vector with indices of cells included in proposition(region) i
% for i=1:length(propositions)
%     ind_obs=find(sum(T.OBS_set==i , 2));  %indices of observables containing prop. i
%     T.props{i}=find(ismember(T.obs,ind_obs)); %searched cells
% end


%%now reduce the above T (PN_Boolean function quotient_T.m)
T_r = rmt_quotient_T(T);

propositions_r=T_r.props;   %T_r's states constructing propositions

%%map some state indices from T to state indices of T_r
R0_r=zeros(size(R0));
for i=1:length(R0)
    for j=1:length(T_r.Q)
        if ismember(R0(i),T_r.Cells{j})
            R0_r(i)=j;
        end
    end
end
