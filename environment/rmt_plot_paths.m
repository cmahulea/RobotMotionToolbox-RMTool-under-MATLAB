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

function message = rmt_plot_paths(Run_cells,new_rob_traj,data,message,ss_replan)
% plot trajectories with animation
% input data:
%   Run_cells - cells crossed by each robot
%   rob_traj - positions of robots thoughout the paths
%   ss_replan - screen-shots every time the trajectories are re-planned
% output data:
%   message - update message with info about results

message = sprintf('%s\nSOLUTION - runs of robots: \n',message);
for j = 1 : size(Run_cells,1)
    message = sprintf('%s\nRobot %d: ',message,j);
    temp = Run_cells(j,:);
    for k = 1 : length(temp)-1
        message = sprintf('%s%d,',message,temp(k));
    end
    message = sprintf('%s%d',message,temp(length(temp)));
end

N_r = length(data.RO);
% find the order of the robots based on their original position, useful for
% when the trajectories are re-planned compute the order of robots
idx_end_val = zeros(1,N_r);
for r = 1:N_r
    end_value = Run_cells(r,end);
    idx_end_val(1,r) = find(Run_cells(r,:) == end_value, 1,'first');
end

[idx_end_val, nr_rob] = sort(idx_end_val(1,:));
idx_end_val = [idx_end_val; nr_rob];

message = sprintf('%s======\n Number of steps for all robots is: %d \n', message, size(Run_cells,2)-1); % from cell i -> cell i+1 is considered one step
message = sprintf('%s===========\n The order of the robots is: ', message);
for i = 1:N_r
    message = sprintf('%s %d ', message, idx_end_val(2,i));
end


%% plot trajectories
% initial figures (empty environment)
%
% name_fig = 'Dummy_BoolSpec.fig';
% init_fig = openfig(name_fig)';

alpha_transparency = 0.5;
color_transparency = {[1,0,0,alpha_transparency], [0,0,1,alpha_transparency], [1,0,1,alpha_transparency],...
    [0,1,0,alpha_transparency], [0,1,1,alpha_transparency], [0,0,0,alpha_transparency], [1,1,0,alpha_transparency],...
    [0.8500 0.3250 0.0980,alpha_transparency],[0.4940 0.1840 0.5560,alpha_transparency],...
    [0.6350 0.0780 0.1840,alpha_transparency],[0 0.4470 0.7410,alpha_transparency],...
    [1,0,0,alpha_transparency], [0,0,1,alpha_transparency], [1,0,1,alpha_transparency],...
    [0,1,0,alpha_transparency], [0,1,1,alpha_transparency], [0,0,0,alpha_transparency], [1,1,0,alpha_transparency],...
    [0.8500 0.3250 0.0980,alpha_transparency],[0.4940 0.1840 0.5560,alpha_transparency],...
    [0.6350 0.0780 0.1840,alpha_transparency],[0 0.4470 0.7410,alpha_transparency]};
hh_v = [];
hh_m = [];
no_cell_plot = 0;
flag_h_text = 0;% parallel movement of the robots
count_replan_text = 0;

% read images
files = dir('examples\Rob_Images\rob*.png');      % as example : only tiff files with "handheld" in the filename
% main loop
idx_img = 1;
for ck = 1:length(files)
    image_name = files(ck).name;
    temp_image = imread(['examples\Rob_Images\',image_name]);
    for i = 1:size(temp_image,1)
    for j = 1:size(temp_image,2)
        if temp_image(i,j,2:3) == 0 & temp_image(i,j,1) == 0
            temp_image(i,j,:) = 255;
        end
    end
   
    end
    % put images in order: r1, r2, etc... (switch r2 - r10)
if ck ~= 2
     Images{idx_img} = temp_image;
     idx_img = idx_img + 1;
else
    aux_temp_img = temp_image;
end
end

Images{end + 1} = aux_temp_img;


for uu = 1:length(new_rob_traj{1})-1
    for rr = 1:length(new_rob_traj)
        % color last 2 cells of each trajectory and add the current
        % position of the robot
        current_cell_traj = Run_cells(rr,uu);
%         if rr == 1
            x1im1 = max(data.T.Vert{current_cell_traj}(1,:));
            x2im1 = min(data.T.Vert{current_cell_traj}(1,:));
            y1im1 = max(data.T.Vert{current_cell_traj}(2,:));
            y2im1 = min(data.T.Vert{current_cell_traj}(2,:));

            hh = image('CData',Images{rr},'XData',[x1im1 x2im1],'YData',[y1im1 y2im1]);

%         else
%         hh = fill(data.T.Vert{current_cell_traj}(1,:),data.T.Vert{current_cell_traj}(2,:),data.rob_plot.line_color{rr},'FaceAlpha',0.2,'EdgeColor',data.rob_plot.line_color{rr},'LineWidth',2);
%         set(hh,'XData', data.T.Vert{current_cell_traj}(1,:), 'YData',data.T.Vert{current_cell_traj}(2,:), 'FaceAlpha', 0.2);
%     end
        hh_marker = plot(mean(data.T.Vert{current_cell_traj}(1,:)),mean( data.T.Vert{current_cell_traj}(2,:)),'Color',data.rob_plot.line_color{rr},...
            'Marker',data.rob_plot.marker{rr},'LineWidth',data.rob_plot.line_width{rr});
        set(hh_marker,'XData', mean(data.T.Vert{current_cell_traj}(1,:)), 'YData',mean( data.T.Vert{current_cell_traj}(2,:)),'Marker',data.rob_plot.marker{rr});
        hh_v{rr} = hh;
        hh_m{rr} = hh_marker;
        if uu == 1 % mark the start point
            plot(new_rob_traj{rr}(1,uu),new_rob_traj{rr}(2,uu),'Color',data.rob_plot.line_color{rr},...
                'Marker',data.rob_plot.marker{rr},'LineWidth',data.rob_plot.line_width{rr});
        elseif uu == size(new_rob_traj{rr},2)-1 % mark the end point
            plot(new_rob_traj{rr}(1,end),new_rob_traj{rr}(2,end),'Color',data.rob_plot.line_color{rr},...
                'Marker',data.rob_plot.marker{rr},'LineWidth',data.rob_plot.line_width{rr},'Color','r');
            
        end
        hh_v{rr} = hh;
    end
    if ~isempty(find(ss_replan == uu,1)) % plot the moments where the replanning is made
        flag_h_text = 1;
        count_replan_text = count_replan_text + 1;
        str_text_static = strcat(num2str(count_replan_text),' times');
        h_text = text(0,data.handle_env.YLim(end) + 0.5,str_text_static);
        
    end
    if uu == 1
        pause % for screen recording
    end
    if uu >= 2
        for kr = 1:length(new_rob_traj)
            plot(new_rob_traj{kr}(1,uu-1:uu),new_rob_traj{kr}(2,uu-1:uu),'Color',color_transparency{kr},'LineWidth',2,'LineStyle','-.');
        end
        if uu == length(new_rob_traj{1}) - 1 % compute the last segment from trajectories
            for kr = 1:length(new_rob_traj)
                plot(new_rob_traj{kr}(1,uu:uu+1),new_rob_traj{kr}(2,uu:uu+1),'Color',color_transparency{kr},'LineWidth',2,'LineStyle','-.');
            end
        end
    end
    drawnow;
    pause(0.5);
    if flag_h_text == 1
        pause(1);
    end
    % update
    for kr = 1:length(new_rob_traj)
        delete(hh_m{kr});
        delete(hh_v{kr});
    end
    if flag_h_text == 1
        delete(h_text);
        flag_h_text = 0;
    end
end

% add text in GUI with the number of replanning
str_text_static = strcat(num2str(count_replan_text),' times');
h_text = text(0,data.handle_env.YLim(end) + 0.5,str_text_static);



end