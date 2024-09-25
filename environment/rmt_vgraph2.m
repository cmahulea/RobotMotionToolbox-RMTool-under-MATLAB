%    This is part of RMTool - Robot Motion Toolbox, for Matlab 2010b or newer.
%
%    Copyright (C) 2016 RMTool developing team, with support of Silvia Magdici. For people, details and citing 
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

% the last version ok -> 12.05.2013


function [trajectory] = rmt_vgraph2(handle,input_variables,seq_obstacles)

Nxi = input_variables(1);
Nx = input_variables(2);
Nyi = input_variables(3);
Ny = input_variables(4);
Nobstacles = input_variables(5);
wheelbase = input_variables(6);
dilation = ceil((wheelbase/2));
%Initial and goal points
xini = input_variables(7);
yini = input_variables(8);
xgoal = input_variables(9);%X-AXIS (0..20)
ygoal = input_variables(10);%Y-AXIS (0..10)
env_bounds=[Nxi,Nx,Nyi,Ny];

axes(handle);
axis(env_bounds);
hold on
grid on


%reading the evolution domain
x_max = Nx;
y_max = Ny;

reg = seq_obstacles;
poly2 = seq_obstacles;
[aa,ba] = size(reg{1});

for i=1:Nobstacles
    if(aa>ba)
        aux = reg{i}';
        aux_poly = expandPolygon(aux',0.3);
        a = aux_poly{1};
        poly2(i) = {a'};
    else
        aux = reg{i};
        aux_poly = expandPolygon(aux',0.3);
        a = aux_poly{1};
        poly2(i) = {a'};
    end 
    drawPolygon(aux_poly);%,'g','LineWidth',2);    
end

reg = poly2;
pols_reg = cell(1,length(reg));
for i = 1 : length(reg)
    pols_reg{i} = polyshape(reg{i}');
end

x_start = xini;
y_start = yini;
x_goal = xgoal;
y_goal = ygoal;
no_of_edges=0;
edges = {};
%find the trajectories from start point to the vertices of the regions
start_point= [x_start,y_start];
%n = Nobstacles;
for i=1:Nobstacles      %for each region
    for j = 1:length(reg{i})  %for each vertice of that region
        point2=reg{i}(:,j)';  % point[x y]
        flag_inters_regiuni=0; % nu exista intersectii
        for k=1:Nobstacles
            [in,~] = intersect(pols_reg{k},[start_point;point2]);%exista intersectie doar cu varful regiunii dreapta start-point -> intersectio-point nu trebuie sa intersecteze oricare alta regiune
            if ~isempty(in)
                flag_inters_regiuni = 1;
                break;
            end
        end
        if(flag_inters_regiuni ~= 1)
            edges{end+1}= [start_point' point2'];
        end
    end
end

%find the trajectories from stop point to the vertices of the regions
stop_point= [x_goal,y_goal];
for i=1:Nobstacles      %for each region
    for j = 1:length(reg{i})  %for each vertice of that region
        point2=reg{i}(:,j)';  % point[x y]
        flag_inters_regiuni=0; % nu exista intersectii
        for k=1:Nobstacles
            [in,~] = intersect(pols_reg{k},[stop_point;point2]);%exista intersectie doar cu varful regiunii dreapta start-point -> intersectio-point nu trebuie sa intersecteze oricare alta regiune
            if ~isempty(in)
                flag_inters_regiuni = 1;
                break;
            end
        end
        if(flag_inters_regiuni ~= 1)
            edges{end+1}= [stop_point' point2'];
        end
    end
end


% edges for every region
for i=1:Nobstacles
    for j = 1:length(reg{i})
        if j < length(reg{i})
            edge_to_check = [reg{i}(:,j)'; reg{i}(:,j+1)'];
        else
            edge_to_check = [reg{i}(:,j)'; reg{i}(:,1)'];
        end
        flag_inside_regiuni=0; % is not insede any region
        for k=1:Nobstacles
            if (k ~= i)
                [~,edge_to_check] = intersect(pols_reg{k},edge_to_check);%exista intersectie doar cu varful regiunii dreapta start-point -> intersectio-point nu trebuie sa intersecteze oricare alta regiune
                if size(edge_to_check,1) >2
                    flag_inside_regiuni = 1;
                    break;
                end
                if isempty(edge_to_check)
                    flag_inside_regiuni = 1;
                    break;
                end
            else
                continue;
            end
        end
        if (flag_inside_regiuni == 0)
            edges{end+1}= edge_to_check';
        end
    end
end

%supporting  edge 
for i=1:Nobstacles-1
    for j = 1 : size(reg{i},2)
        first_point = reg{i}(:,j)';
        for k= i+1:Nobstacles
            for l = 1 : size(reg{k},2)
                second_point = reg{k}(:,l)';
                flag_inters_regiuni = 0;
                for m = 1 : Nobstacles
                    [in,~] = intersect(pols_reg{m},[first_point;second_point]);%exista intersectie doar cu varful regiunii dreapta start-point -> intersectio-point nu trebuie sa intersecteze oricare alta regiune
                    if ~isempty(in)
                        flag_inters_regiuni = 1;
                        break;
                    end
                end
                if (flag_inters_regiuni ~= 1)
                    edges{end+1} = [first_point' second_point'];
                end
            end
            %-----------------------------------------
        end
    end
end

%from start to goal
flag_inters_regiuni=0; % nu exista intersectii
for i=1:Nobstacles
    [in,~] = intersect(pols_reg{i},[start_point;stop_point]);%exista intersectie doar cu varful regiunii dreapta start-point -> intersectio-point nu trebuie sa intersecteze oricare alta regiune
    if ~isempty(in)
        flag_inters_regiuni = 1;
        break;
    end
end
if (flag_inters_regiuni ~= 1) % exista drum intre start si stop
    edges{end+1} = [[x_start,y_start]'  [x_goal,y_goal]'];
end

for i=1:length(edges)
    plot(edges{i}(1,:), edges{i}(2,:), 'Color',[.8 .8 .8]);
end


%---------------------------------------------------------------
%    26.05.2013
% --------------------------------------------------------------

nrElem=1;
VfGraphs{nrElem}= edges{1}(:,1);
for i=2: length(edges)
    flag=0;
    for j=1:length(VfGraphs)
        if(VfGraphs{j} == edges{i}(:,1))
            flag=1;
        end
    end
    if(flag == 0)
        nrElem = nrElem+1;
        VfGraphs{nrElem} = edges{i}(:,1);
    end
end

% for i=1: length(edges)
%     flag=0;
%     for j=1:length(VfGraphs)
%         if(VfGraphs{j} == edges{i}(:,2))
%             flag=1;
%         end
%     end
%     if(flag == 0)
%         nrElem = nrElem+1;
%         VfGraphs{nrElem} = edges{i}(:,2);
%     end
% end


%for i=1:length(VfGraphs)
%plot(VfGraphs{i}(1,:), VfGraphs{i}(2,:), '-yo')
%end

%In vectorul VfGraphs se afla coordonatele tuturor varfurilor din graf.

%determin index start
start_p=start_point';
stop_p=stop_point';
for i=1:length(VfGraphs)
    if(VfGraphs{i}==start_p)
        indexStart = i;
    end
    if(VfGraphs{i}==stop_p)
        indexStop = i;
    end
end

% cost = matricea de adiacenta

len = length (VfGraphs);
%cost = eye(len)/100;
cost = zeros(len);

for i=1:length(edges)
    %parcurg fiecare edge
    index_1 = returnIndex(edges{i}(:,1), VfGraphs);
    index_2 = returnIndex(edges{i}(:,2), VfGraphs);
    if(index_1 ~=0 && index_2 ~=0)
        cost(index_1, index_2)=norm(edges{i}(:,1)-edges{i}(:,2));
       % cost(index_2, index_1)=norm(edges{i}(:,1)-edges{i}(:,2));
       cost(index_2, index_1) = cost(index_1, index_2); 
    end
%     if((index_1 ==0 ) || (index_2 ==0 ))
%         i
%         index_1
%         index_2
%         
%     end
end

%clc
[path, cost_path] = find_path(cost,indexStart,indexStop);
%disp('VfGraphs:');
VfGraphs{path};
%figure(1)
%figure;hold on;
%hold on
% for i=1:length(path)-1
%     p1 = [VfGraphs{path(i)}(1,:) VfGraphs{path(i)}(2,:)];
%     p2 = [VfGraphs{path(i+1)}(1,:) VfGraphs{path(i+1)}(2,:)];
%     drawEdge(p1,p2,'g','LineWidth', 2);    
% %plot(VfGraphs{i}(1,:),VfGraphs{i}(2,:), '*y')
% end


lon = length(path);
trajectory = zeros(lon,2);
trajectory(1,1) = VfGraphs{path(1)}(1,:);
trajectory(1,2) = VfGraphs{path(1)}(2,:);
for(i=1:lon-1)
    p1 = [VfGraphs{path(i)}(1,:) VfGraphs{path(i)}(2,:)];
    p2 = [VfGraphs{path(i+1)}(1,:) VfGraphs{path(i+1)}(2,:)];    
    drawEdge(p1,p2,'r','LineWidth', 2);       
    trajectory(i,1) = VfGraphs{path(i)}(1,:);
    trajectory(i,2) = VfGraphs{path(i)}(2,:);
end
trajectory(lon,1) = VfGraphs{path(lon)}(1,:);
trajectory(lon,2) = VfGraphs{path(lon)}(2,:);

%disp('V-graph ends.');
