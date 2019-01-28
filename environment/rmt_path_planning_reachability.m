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
%   First version released on November, 2018.
%   Last modification November 10, 2018.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function rmt_path_planning_reachability
%Path-planning with reachability specification and transition system models (graph based approach)

data = get(gcf,'UserData');
planning_approach = get(findobj(gcf,'Tag','pathpoints'),'Value'); %planning_approach= 1 - cell descomposition; 2 - visibility graph; 3 - Voronoi
cla(data.handle_ori);
cla(data.handle_vel);
cla(data.handle_ang);
if ~isfield(data,'Nobstacles')
    error('Number of obstacles is zero!');
end
if (data.Nobstacles==0)
    uiwait(errordlg(sprintf('\nDefine an environment first!'),'Robot Motion Toolbox','modal'));
    return;
end
for i=1:data.Nobstacles
    aux = data.obstacles{i};
    [as, bs] = size(data.obstacles{1});
    if(as > bs)
        aux = aux';
    end
    obstaclesCD(i) = {aux};
end

switch planning_approach
    case 1%cell decomposition
        if (get(findobj(gcf,'Tag','triang'),'Value') == 1)
            [C,adj,mid_X,mid_Y,com_F]=rmt_triangular_decomposition(obstaclesCD,data.frame_limits);   %triangular decomposition
        elseif (get(findobj(gcf,'Tag','rect'),'Value') == 1)
            [C,adj,mid_X,mid_Y,com_F]=rmt_rectangular_decomposition(obstaclesCD,data.frame_limits);   %rectangular decomposition
        elseif (get(findobj(gcf,'Tag','poly'),'Value') == 1)
            [C,adj,mid_X,mid_Y,com_F]=rmt_polytopal_decomposition(obstaclesCD,data.frame_limits);   %polytopal decomposition
        elseif (get(findobj(gcf,'Tag','trapez'),'Value') == 1)
            [C,adj,mid_X,mid_Y,com_F]=rmt_trapezoidal_decomposition(obstaclesCD,data.frame_limits);   %trapezoidal decomposition
        end
        cla(data.handle_env);
        axes(data.handle_env);
        rmt_plot_environment(obstaclesCD,data.frame_limits,C,'c');
        for k = 1 : length(data.initial)
            plot(data.initial{k}(1),data.initial{k}(2),'pw',...
                'Markersize',13, 'Color', 'k');
        end
        for k = 1 : length(data.final)
            plot(data.final{k}(1),data.final{k}(2),'pw',...
                'Markersize',13, 'Color', 'b');
        end
        set(data.handle_env,'xlim',[data.frame_limits(1) data.frame_limits(2)],'ylim',[data.frame_limits(3) data.frame_limits(4)],'XGrid','on','YGrid','on');
        grid on;
        %assign weight on the arcs if MPC is not chosen
        if (get(findobj(gcf,'Tag','waypoints'),'Value') ~= 5)
            s = get(findobj(gcf,'Tag','weights'),'Value');
            switch s
                case 2 %distance between the centroids
                    for i = 1 : size(adj,1)-1
                        adj(i,i) = 0.0001; %for selfloops add a very small cost
                        for j = i+1 : size(adj,1)
                            if (adj(i,j)~=0)
                                adj(i,j) = norm(mean(C{i},2)-mean(C{j},2));
                                adj(j,i) = adj(i,j);
                            end
                        end
                    end
                case 3 %cost(ci,cj) = norm(centr(ci)-mid(ci,cj))
                    for i = 1 : size(adj,1)
                        adj(i,i) = 0.0001; %for selfloops add a very small cost
                        for j = 1 : size(adj,1)
                            if ((j ~=i) && (adj(i,j)~=0))
                                temp = com_F{i,j};
                                adj(i,j) = norm(mean(C{i},2)-norm(temp(:,1)-temp(:,2)));
                            end
                        end
                    end
                case 4 %cost(ci,cj) = sum(c_h, c_h~=c_j, norm(mid(c_h,c_i)?mid(c_i,c_j))/(number of neighbors of c1 - 1)
                    for i = 1 : size(adj,1)
                        adj(i,i) = 0.0001; %for selfloops add a very small cost
                        for j = 1 : size(adj,1)
                            if ((j ~=i) && (adj(i,j)~=0))
                                temp = setdiff(find(adj(:,i)),[i,j]);
                                adj(i,j) = 0;
                                for k = 1 : length(temp)
                                    temp2 = com_F{temp(k),i};
                                    temp3 = com_F{j,i};
                                    adj(i,j) = adj(i,j)+...
                                        norm( norm(temp2(:,1) - temp2(:,2))...
                                        - norm(norm(temp3(:,1) - temp3(:,2))));
                                end
                                adj(i,j) = adj(i,j) / length(temp);
                            end
                        end
                    end
            end
        end
        temp1 = data.initial{1};
        temp2 = data.final{1};
        if (get(findobj(gcf,'Tag','waypoints'),'Value') == 1) %middle points
            tic;
            [traj, travel_dist, path_cells, cost_path] = rmt_find_trajectory(C,adj,mid_X,mid_Y,...
                temp1(1),temp1(2),temp2(1),temp2(2),obstaclesCD);
            plot(traj(1,:),traj(2,:),'r','LineWidth',2);
            data.trajectory = traj;
            message = sprintf('Travelled distance via middle points: %.2f.\n\n Time to compute trajectory %.2f segundos',...
                travel_dist,toc);
            uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
        elseif (get(findobj(gcf,'Tag','waypoints'),'Value') == 2)  %norm 1
            %function 1-norm
            safe_dist=0.5;  %should be smaller than half of shortest traversed segment
            tic;
            [traj, travel_dist, path_cells, cost_path] = rmt_find_trajectory(C,adj,mid_X,mid_Y,...
                temp1(1),temp1(2),temp2(1),temp2(2),obstaclesCD);
            start_p = temp1;
            goal_p = temp2;
            [traj_norm_one, dist_norm_one] = rmt_optimize_traj_norm_one(com_F,...
                path_cells,start_p,goal_p,safe_dist);  %via LP
            plot(traj_norm_one(1,:),traj_norm_one(2,:),'r','LineWidth',2);
            data.trajectory = traj_norm_one;
            message = sprintf('Travelled distance via Norm-1: %.2f.\n\nTime to compute the trajectory: %.2f.',...
                dist_norm_one,toc);
            uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
        elseif (get(findobj(gcf,'Tag','waypoints'),'Value') == 3)  %norm 2
            safe_dist=0.5;  %should be smaller than half of shortest traversed segment
            tic;
            [traj, travel_dist, path_cells, cost_path] = rmt_find_trajectory(C,adj,mid_X,mid_Y,...
                temp1(1),temp1(2),temp2(1),temp2(2),obstaclesCD);
            start_p = temp1;
            goal_p = temp2;
            [traj_norm_2_sq, dist_norm_2_sq] = rmt_optimize_traj_norm_two_sq(com_F,path_cells,start_p,goal_p,safe_dist);  %via QP
            data.trajectory = traj_norm_2_sq;
            plot(traj_norm_2_sq(1,:),traj_norm_2_sq(2,:),'r','LineWidth',2);
            message = sprintf('Travelled distance via Norm-2: %.2f.\n\nTime to compute the trajectory %.2f.',...
                dist_norm_2_sq,toc);
            uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
        elseif (get(findobj(gcf,'Tag','waypoints'),'Value') == 4)  %norm inf.
            safe_dist=0.5;  %should be smaller than half of shortest traversed segment
            tic;
            [traj, travel_dist, path_cells, cost_path] = rmt_find_trajectory(C,adj,mid_X,mid_Y,...
                temp1(1),temp1(2),temp2(1),temp2(2),obstaclesCD);
            start_p = temp1;
            goal_p = temp2;
            [traj_norm_inf, dist_norm_inf] = rmt_optimize_traj_norm_inf(com_F,path_cells,start_p,goal_p,safe_dist);  %via LP
            data.trajectory = traj_norm_inf;
            plot(traj_norm_inf(1,:),traj_norm_inf(2,:),'r','LineWidth',2);
            message = sprintf('Travelled distance via Norm-Inf.: %.2f.\n\nTime to compute the trajectory: %.2f',...
                dist_norm_inf,toc);
            uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
        elseif (get(findobj(gcf,'Tag','waypoints'),'Value') == 5)  %MPC
            data = get(gcf,'UserData');
            prompt = {'Safety distance:','N:','Number of intermediate points on common edges:'};
            dlg_title = 'Robot Motion Toolbox';
            num_lines = 1;
            defaultans = {num2str(data.waypointsMPC.safe_dist),num2str(data.waypointsMPC.N),...
                num2str(data.waypointsMPC.intermPoints)};
            input_user = inputdlg(prompt,dlg_title,num_lines,defaultans);
            data.waypointsMPC.safe_dist = str2num(char(input_user(1)));   %should be smaller than half of shortest traversed segment
            data.waypointsMPC.N = str2num(char(input_user(2))); % Receding horizon
            data.waypointsMPC.intermPoints = str2num(char(input_user(3)));
            set(gcf,'UserData',data);
            tic;
            [traj, travel_dist] = rmt_optimize_traj_mpc(C,adj,com_F,...
                data.waypointsMPC.safe_dist,data.waypointsMPC.intermPoints,...
                data.waypointsMPC.N, temp1(1),temp1(2),temp2(1),temp2(2),obstaclesCD);
            for i = 1 : size(traj,2)-1
                plot([traj(1,i) traj(1,i+1)],[traj(2,i) traj(2,i+1)],'r');
            end
            message = sprintf('Travelled distance via MPC: %.2f.\n\nTime to compute the trajectory: %.2f',...
                travel_dist,toc);
            uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
        end
        set(data.handle_env,'xlim',[data.frame_limits(1) data.frame_limits(2)],'ylim',[data.frame_limits(3) data.frame_limits(4)],'XGrid','on','YGrid','on');
    case 2%visibility graph
        input_variables = zeros(8,1);
        input_variables(1) = data.frame_limits(1);
        input_variables(2) = data.frame_limits(2);
        input_variables(3) = data.frame_limits(3);
        input_variables(4) = data.frame_limits(4);
        input_variables(5) = length(data.obstacles);
        input_variables(6) = data.control.wheel_base;%eval(get(findobj(gcf,'Tag','wheelbase'),'String'));
        temp1 = data.initial{1};
        temp2 = data.final{1};
        input_variables(7) = temp1(1);
        input_variables(8) = temp1(2);
        input_variables(9) = temp2(1);
        input_variables(10) = temp2(2);
        cla(data.handle_env);%new
        axes(data.handle_env);
        rmt_plot_environment(data.obstacles,data.frame_limits);
        plot(temp1(1),temp1(2),'pw','Markersize',13, 'Color', 'k');
        plot(temp2(1),temp2(2),'pw','Markersize',13, 'Color', 'b');
        grid on;
        %traj = rmt_visibility_graph_new2(data.handle_env,input_variables,data.map,data.obstacles);
        traj = rmt_vgraph2(data.handle_env,input_variables,data.obstacles);
        %plot(traj(1,:),traj(2,:),'--r','LineWidth',2);%new
        %set(data.handle_env,'xlim',[data.frame_limits(1) data.frame_limits(2)],'ylim',[data.frame_limits(3) data.frame_limits(4)],'XGrid','on','YGrid','on');
        %[trajectory] = rmt_vgraph2(handle,input_variables,seq_obstacles)
        data.trajectory = traj';
        set(data.handle_env,'xlim',[data.frame_limits(1) data.frame_limits(2)],'ylim',[data.frame_limits(3) data.frame_limits(4)],'XGrid','on','YGrid','on');
    case 3%voronoi
        cla(data.handle_env);%new
        axes(data.handle_env);
        rmt_plot_environment(data.obstacles,data.frame_limits);
        temp1 = data.initial{1};
        temp2 = data.final{1};
        plot(temp1(1),temp1(2),'pw','Markersize',13, 'Color', 'k');
        plot(temp2(1),temp2(2),'pw','Markersize',13, 'Color', 'b');
        grid on;
        limits = data.frame_limits;
        whe = eval(get(findobj(gcf,'Tag','wheelbase'),'String'));
        [X_Total_points,Y_Total_points, ...
            All_cells_Number, Cell_start, X1] = rmt_voronoi_epsi(data.handle_env,data.Nobstacles,limits,whe*0.5,...
            data.epsilonvoronoi,data.obstacles);
        data.X_Total_points = X_Total_points;
        data.Y_Total_points = Y_Total_points;
        data.All_cells_Number = All_cells_Number;
        data.Cell_start = Cell_start;
        data.X1 = X1;
        traj = rmt_get_voronoi(data.handle_env, data.frame_limits, (data.Nobstacles+1),...
            temp1, temp2,data.X_Total_points, data.Y_Total_points, ...
            data.All_cells_Number, data.Cell_start, data.X1);
        %traj = rmt_visibility_graph(data.handle_env,input_variables,data.map,data.obstacles);
        data.trajectory = traj';
end
set(gcf,'UserData',data);
return